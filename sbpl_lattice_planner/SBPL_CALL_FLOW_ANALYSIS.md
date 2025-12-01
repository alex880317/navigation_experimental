# SBPL Lattice Planner 調用流程與 Char 類型問題分析

## 一、算法調用入口

### 1.1 ROS Plugin 註冊機制

SBPL Lattice Planner 通過 ROS plugin 機制註冊為全局規劃器：

```53:53:src/navigation_experimental/sbpl_lattice_planner/src/sbpl_lattice_planner.cpp
PLUGINLIB_EXPORT_CLASS(sbpl_lattice_planner::SBPLLatticePlanner, my_nav_core::BaseGlobalPlanner)
```

### 1.2 move_base 調用流程

**調用鏈：**
```
move_base (my_move_base.cpp)
  └─> planner_->makePlan(start, goal, plan)  [第 501 行]
      └─> SBPLLatticePlanner::makePlan()  [sbpl_lattice_planner.cpp 第 297 行]
```

**關鍵代碼位置：**
- **move_base 調用入口**：`src/my_nav/move_base/src/my_move_base.cpp:501`
- **SBPL 實現入口**：`src/navigation_experimental/sbpl_lattice_planner/src/sbpl_lattice_planner.cpp:297`

### 1.3 初始化流程

當 move_base 啟動時：

```129:135:src/my_nav/move_base/src/my_move_base.cpp
    try {
      planner_ = bgp_loader_.createInstance(global_planner);
      planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
      exit(1);
    }
```

這會調用 `SBPLLatticePlanner::initialize()` 函數。

## 二、整個算法的計算過程

### 2.1 初始化階段 (`initialize()`)

**位置**：`sbpl_lattice_planner.cpp:100-236`

**主要步驟：**

1. **讀取參數**（第 106-124 行）
   - `planner_type_`: ARAPlanner 或 ADPlanner
   - `allocated_time_`: 規劃時間限制
   - `initial_epsilon_`: 初始 epsilon 值
   - `primitive_filename_`: 運動基元文件路徑

2. **設置成本參數**（第 130-135 行）
   ```cpp
   lethal_obstacle_ = (uint8_t) lethal_obstacle;
   inscribed_inflated_obstacle_ = lethal_obstacle_-1;
   sbpl_cost_multiplier_ = (uint8_t) (costmap_2d::INSCRIBED_INFLATED_OBSTACLE/inscribed_inflated_obstacle_ + 1);
   ```

3. **初始化 SBPL 環境**（第 192-214 行）
   - 創建 `EnvironmentNAVXYTHETALAT` 環境
   - 調用 `env_->InitializeEnv()` 初始化環境
   - 遍歷所有 costmap 單元格，調用 `env_->UpdateCost()` 更新成本

4. **創建規劃器**（第 216-227 行）
   - 根據 `planner_type_` 創建 `ARAPlanner` 或 `ADPlanner`

### 2.2 規劃階段 (`makePlan()`)

**位置**：`sbpl_lattice_planner.cpp:297-493`

**完整計算流程：**

#### 步驟 1: 檢查並重新初始化（如果需要）（第 305-329 行）
```cpp
if (current_env_width_ != costmap_ros_->getCostmap()->getSizeInCellsX() ||
    current_env_height_ != costmap_ros_->getCostmap()->getSizeInCellsY()) {
  // 重新初始化
}
```

#### 步驟 2: 設置起點和終點（第 333-360 行）
```cpp
// 計算起始角度
double theta_start = 2 * atan2(start.pose.orientation.z, start.pose.orientation.w);
double theta_goal = 2 * atan2(goal.pose.orientation.z, goal.pose.orientation.w);

// 設置起點（第 339 行）
int ret = env_->SetStart(
    start.pose.position.x - costmap_ros_->getCostmap()->getOriginX(), 
    start.pose.position.y - costmap_ros_->getCostmap()->getOriginY(), 
    theta_start);
planner_->set_start(ret);

// 設置終點（第 351 行）
int ret = env_->SetGoal(
    goal.pose.position.x - costmap_ros_->getCostmap()->getOriginX(), 
    goal.pose.position.y - costmap_ros_->getCostmap()->getOriginY(), 
    theta_goal);
planner_->set_goal(ret);
```

#### 步驟 3: 更新 Costmap（第 367-395 行）
```cpp
for(unsigned int ix = 0; ix < costmap_ros_->getCostmap()->getSizeInCellsX(); ix++) {
  for(unsigned int iy = 0; iy < costmap_ros_->getCostmap()->getSizeInCellsY(); iy++) {
    uint8_t oldCost = env_->GetMapCost(ix,iy);  // 從 SBPL 環境獲取舊成本
    uint8_t newCost = costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix,iy));  // 從 ROS costmap 獲取新成本
    
    if(oldCost == newCost) continue;  // 如果成本未改變，跳過
    
    // 更新 SBPL 環境中的成本
    env_->UpdateCost(ix, iy, costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix,iy)));
    
    // 記錄改變的單元格
    nav2dcell_t nav2dcell;
    nav2dcell.x = ix;
    nav2dcell.y = iy;
    changedcellsV.push_back(nav2dcell);
  }
}
```

#### 步驟 4: 通知規劃器成本已改變（第 397-410 行）
```cpp
if(!changedcellsV.empty()){
  StateChangeQuery* scq = new LatticeSCQ(env_, changedcellsV);
  planner_->costs_changed(*scq);  // 通知規劃器成本已改變
  delete scq;
}

if(allCount > force_scratch_limit_)
  planner_->force_planning_from_scratch();  // 如果改變太多，強制重新規劃
```

#### 步驟 5: 執行規劃（第 412-433 行）
```cpp
// 設置規劃參數
planner_->set_initialsolution_eps(initial_epsilon_);
planner_->set_search_mode(false);

// 執行規劃算法（核心調用）
vector<int> solution_stateIDs;
int solution_cost;
int ret = planner_->replan(allocated_time_, &solution_stateIDs, &solution_cost);
```

**這是算法的核心入口點！** `planner_->replan()` 會調用 SBPL 庫內部的 ARA* 或 AD* 算法。

#### 步驟 6: 轉換路徑格式（第 437-492 行）
```cpp
// 將狀態 ID 路徑轉換為 (x, y, theta) 路徑
env_->ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &sbpl_path);

// 轉換為 ROS 格式的 PoseStamped 路徑
for(unsigned int i=0; i<sbpl_path.size(); i++){
  pose.pose.position.x = sbpl_path[i].x + costmap_ros_->getCostmap()->getOriginX();
  pose.pose.position.y = sbpl_path[i].y + costmap_ros_->getCostmap()->getOriginY();
  // ... 設置角度等
  plan.push_back(pose);
}
```

### 2.3 成本轉換函數 (`costMapCostToSBPLCost()`)

**位置**：`sbpl_lattice_planner.cpp:240-253`

這個函數將 ROS costmap 的成本值（0-255）轉換為 SBPL 使用的成本值：

```cpp
uint8_t SBPLLatticePlanner::costMapCostToSBPLCost(uint8_t newcost){
  if(newcost == costmap_2d::LETHAL_OBSTACLE || (!allow_unknown_ && newcost == costmap_2d::NO_INFORMATION))
    return lethal_obstacle_;  // 致命障礙物
  else if(newcost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    return inscribed_inflated_obstacle_;  // 內切膨脹障礙物
  else if(newcost == 0 || newcost == costmap_2d::NO_INFORMATION)
    return 0;  // 自由空間
  else {
    uint8_t sbpl_cost = newcost / sbpl_cost_multiplier_;  // 縮放成本值
    if (sbpl_cost == 0)
      sbpl_cost = 1;  // 確保非零
    return sbpl_cost;
  }
}
```

## 三、Char Signed/Unsigned 問題分析

### 3.1 問題根源

**關鍵問題點：**

1. **Costmap 返回類型**：`costmap_2d::Costmap2D::getCost()` 返回 `unsigned char`（第 116 行）
2. **SBPL 接口類型**：SBPL 庫的 `GetMapCost()` 和 `UpdateCost()` 可能使用 `char` 類型
3. **ARM 架構特殊性**：在 ARM 架構上，`char` 預設可能是 `unsigned char`

### 3.2 受影響的代碼位置

#### 位置 1: 成本比較（第 370-373 行）
```cpp
uint8_t oldCost = env_->GetMapCost(ix,iy);  // SBPL 返回的可能是 char
uint8_t newCost = costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix,iy));  // ROS 返回 unsigned char

if(oldCost == newCost) continue;  // 如果類型不匹配，比較可能出錯
```

**問題：**
- 如果 `env_->GetMapCost()` 內部返回 `char`（signed），而 `newCost` 是 `uint8_t`（unsigned）
- 當 `oldCost` 的值 > 127 時，它會被解釋為負數（例如 200 變成 -56）
- 比較 `-56 == 200` 會失敗，導致不必要的更新

#### 位置 2: 成本轉換函數（第 240-253 行）
```cpp
uint8_t SBPLLatticePlanner::costMapCostToSBPLCost(uint8_t newcost){
  // ...
  uint8_t sbpl_cost = newcost / sbpl_cost_multiplier_;  // 除法運算
  // ...
}
```

**問題：**
- 如果 `sbpl_cost_multiplier_` 在計算時發生溢出或類型轉換問題
- 第 134 行的計算：`sbpl_cost_multiplier_ = (uint8_t) (costmap_2d::INSCRIBED_INFLATED_OBSTACLE/inscribed_inflated_obstacle_ + 1);`
- 如果 `inscribed_inflated_obstacle_` 為 0 或很小，可能導致除零或溢出

#### 位置 3: SBPL 庫內部接口

**雖然代碼中使用了 `uint8_t`，但問題可能在於：**

1. **SBPL 庫的接口定義**：SBPL 庫的 `GetMapCost()` 和 `UpdateCost()` 可能定義為：
   ```cpp
   char GetMapCost(int x, int y);  // 返回 char（可能是 signed）
   void UpdateCost(int x, int y, char cost);  // 接受 char 參數
   ```

2. **類型轉換問題**：
   ```cpp
   uint8_t oldCost = env_->GetMapCost(ix,iy);  // 如果 GetMapCost 返回 char，這裡會發生隱式轉換
   ```
   - 如果 `GetMapCost()` 返回 `char`（signed），值 > 127 會被解釋為負數
   - 賦值給 `uint8_t` 時會發生符號擴展，導致值錯誤

### 3.3 為什麼改成 uint8_t 後還是有問題？

**原因分析：**

1. **SBPL 庫內部仍使用 char**
   - 即使 wrapper 代碼使用 `uint8_t`，SBPL 庫內部可能仍使用 `char` 類型存儲成本
   - 當從 SBPL 庫讀取成本時，如果庫返回 `char`，值 > 127 會被解釋為負數

2. **接口類型不匹配**
   - `env_->GetMapCost(ix,iy)` 的返回類型可能不是 `uint8_t`
   - 如果返回 `char`，即使賦值給 `uint8_t`，在比較時仍可能出問題

3. **編譯器優化問題**
   - 在某些編譯器優化下，`char` 和 `uint8_t` 的比較可能被優化為不同的指令
   - ARM 架構上，符號擴展的行為可能與 x86 不同

4. **成本值的範圍問題**
   - ROS costmap 使用 0-255 範圍（`unsigned char`）
   - 如果 SBPL 內部使用 `signed char`（-128 到 127），值 > 127 會溢出

### 3.4 具體問題場景

**場景 1：成本值 > 127**
```cpp
// 假設 SBPL 內部存儲成本為 char（signed）
char sbpl_cost = 200;  // 在 signed char 中，200 被解釋為 -56

// 當讀取時
uint8_t oldCost = env_->GetMapCost(ix,iy);  // 如果返回 char，200 變成 -56
// 然後 -56 被轉換為 uint8_t，變成 200（因為 uint8_t 是無符號的）

// 但問題在於比較時
uint8_t newCost = 200;  // 從 ROS costmap 獲取
if(oldCost == newCost) continue;  // 200 == 200，應該相等

// 但如果 SBPL 內部比較時使用 signed char
// SBPL 內部：-56 != 200（因為類型不同）
```

**場景 2：成本轉換時的溢出**
```cpp
// 第 248 行
uint8_t sbpl_cost = newcost / sbpl_cost_multiplier_;

// 如果 newcost = 200, sbpl_cost_multiplier_ = 10
// sbpl_cost = 20（正確）

// 但如果 newcost = 255, sbpl_cost_multiplier_ = 1
// sbpl_cost = 255（可能超出 SBPL 的預期範圍）
```

## 四、解決方案建議

### 4.1 確保類型一致性

在 `sbpl_lattice_planner.cpp` 中添加明確的類型轉換：

```cpp
// 在 makePlan() 中，第 370 行修改為：
uint8_t oldCost = static_cast<uint8_t>(static_cast<unsigned char>(env_->GetMapCost(ix,iy)));
```

### 4.2 檢查 SBPL 庫接口

確認 SBPL 庫的接口定義，如果使用 `char`，建議：
1. 修改 SBPL 庫使用 `unsigned char` 或 `uint8_t`
2. 或者在 wrapper 中添加類型轉換層

### 4.3 添加調試輸出

在關鍵位置添加調試信息：

```cpp
uint8_t oldCost = env_->GetMapCost(ix,iy);
uint8_t newCost = costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix,iy));

// 添加調試
if (oldCost > 127 || newCost > 127) {
  ROS_DEBUG("High cost value detected: oldCost=%u, newCost=%u", oldCost, newCost);
}
```

### 4.4 使用編譯選項

已經在 `CMakeLists.txt` 中添加了 `-fsigned-char`，但這只影響當前編譯單元。如果 SBPL 庫是預編譯的，需要重新編譯 SBPL 庫。

## 五、關鍵代碼位置總結

| 功能 | 文件位置 | 行號 |
|------|---------|------|
| Plugin 註冊 | `sbpl_lattice_planner.cpp` | 53 |
| 初始化入口 | `sbpl_lattice_planner.cpp` | 100 |
| 規劃入口 | `sbpl_lattice_planner.cpp` | 297 |
| move_base 調用 | `my_move_base.cpp` | 501 |
| 成本轉換 | `sbpl_lattice_planner.cpp` | 240 |
| 成本更新循環 | `sbpl_lattice_planner.cpp` | 367-395 |
| 規劃算法調用 | `sbpl_lattice_planner.cpp` | 421 |
| 路徑轉換 | `sbpl_lattice_planner.cpp` | 439 |

## 六、調試建議

1. **添加詳細日誌**：在成本比較和轉換處添加日誌
2. **檢查 SBPL 庫源碼**：確認 `GetMapCost()` 和 `UpdateCost()` 的實際類型
3. **使用 GDB 調試**：在關鍵位置設置斷點，檢查實際的類型轉換
4. **測試不同成本值**：特別測試 > 127 的成本值


