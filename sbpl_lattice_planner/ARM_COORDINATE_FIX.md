# SBPL Lattice Planner ARM 架構座標轉換修復方案

## 問題描述

在 Jetson (ARM 架構) 上，SBPL Lattice Planner 只有第三象限 (x < 0, y < 0) 能成功規劃，其他象限失敗。

## 根本原因分析

### 1. 座標轉換問題

在 `sbpl_lattice_planner.cpp` 第 339 和 351 行，座標轉換使用：
```cpp
env_->SetStart(start.pose.position.x - costmap_ros_->getCostmap()->getOriginX(), 
               start.pose.position.y - costmap_ros_->getCostmap()->getOriginY(), 
               theta_start);
```

**問題：**
- 直接減去 origin 可能導致負座標處理錯誤
- ARM 架構上負數的取模運算 (`%`) 行為可能與 x86 不同
- SBPL 內部可能使用 `unsigned int` 或 `int` 處理座標，導致負數溢出

### 2. 型別轉換問題

ARM 架構上：
- `char` 預設可能是 `unsigned char`（已通過 `-fsigned-char` 修正）
- 但 `int` 和 `unsigned int` 的轉換在負數時可能出問題
- 座標索引計算可能使用 `unsigned` 類型，導致負座標被錯誤解釋

### 3. 可能的修復方案

#### 方案 A: 使用 worldToMap 進行座標轉換（推薦）

修改 `sbpl_lattice_planner.cpp` 的 `makePlan` 函數：

```cpp
// 在 makePlan 函數中，替換座標轉換邏輯
unsigned int start_mx, start_my;
unsigned int goal_mx, goal_my;

if (!costmap_ros_->getCostmap()->worldToMap(
    start.pose.position.x, start.pose.position.y, start_mx, start_my)) {
  ROS_ERROR("Failed to convert start pose to map coordinates");
  return false;
}

if (!costmap_ros_->getCostmap()->worldToMap(
    goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my)) {
  ROS_ERROR("Failed to convert goal pose to map coordinates");
  return false;
}

// 將 map 座標轉換為世界座標（相對於 origin）
double start_wx, start_wy;
double goal_wx, goal_wy;
costmap_ros_->getCostmap()->mapToWorld(start_mx, start_my, start_wx, start_wy);
costmap_ros_->getCostmap()->mapToWorld(goal_mx, goal_my, goal_wx, goal_wy);

// 計算相對於 origin 的座標
double start_x_rel = start.pose.position.x - costmap_ros_->getCostmap()->getOriginX();
double start_y_rel = start.pose.position.y - costmap_ros_->getCostmap()->getOriginY();
double goal_x_rel = goal.pose.position.x - costmap_ros_->getCostmap()->getOriginX();
double goal_y_rel = goal.pose.position.y - costmap_ros_->getCostmap()->getOriginY();

// 添加調試輸出
ROS_DEBUG("[SBPL] Start: world=(%f,%f) map=(%u,%u) rel=(%f,%f)", 
          start.pose.position.x, start.pose.position.y, 
          start_mx, start_my, start_x_rel, start_y_rel);
ROS_DEBUG("[SBPL] Goal: world=(%f,%f) map=(%u,%u) rel=(%f,%f)", 
          goal.pose.position.x, goal.pose.position.y, 
          goal_mx, goal_my, goal_x_rel, goal_y_rel);

try {
  int ret = env_->SetStart(start_x_rel, start_y_rel, theta_start);
  // ... 其餘代碼
}
```

#### 方案 B: 添加型別安全的座標轉換函數

在 `sbpl_lattice_planner.h` 中添加：

```cpp
private:
  // ARM 安全的座標轉換函數
  bool safeWorldToSBPLCoords(double wx, double wy, 
                             double& sbpl_x, double& sbpl_y);
```

在 `sbpl_lattice_planner.cpp` 中實現：

```cpp
bool SBPLLatticePlanner::safeWorldToSBPLCoords(double wx, double wy, 
                                               double& sbpl_x, double& sbpl_y) {
  // 使用 costmap 的 worldToMap 確保座標在有效範圍內
  unsigned int mx, my;
  if (!costmap_ros_->getCostmap()->worldToMap(wx, wy, mx, my)) {
    return false;
  }
  
  // 轉換為相對於 origin 的座標（確保型別安全）
  double origin_x = costmap_ros_->getCostmap()->getOriginX();
  double origin_y = costmap_ros_->getCostmap()->getOriginY();
  
  sbpl_x = wx - origin_x;
  sbpl_y = wy - origin_y;
  
  // 驗證結果（防止溢出）
  if (sbpl_x < -1e6 || sbpl_x > 1e6 || sbpl_y < -1e6 || sbpl_y > 1e6) {
    ROS_WARN("[SBPL] Coordinate out of range: (%f, %f)", sbpl_x, sbpl_y);
    return false;
  }
  
  return true;
}
```

#### 方案 C: 增強編譯選項

在 `CMakeLists.txt` 中添加更多 ARM 兼容性選項：

```cmake
# 增強 ARM 兼容性編譯選項
if(CMAKE_COMPILER_IS_GNUCXX AND CMAKE_SYSTEM_PROCESSOR MATCHES "arm|ARM|aarch64")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsigned-char")
    # 防止浮點運算優化導致的精度問題
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffloat-store")
    # 防止嚴格別名優化問題
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-strict-aliasing")
    # 確保整數運算的一致性
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fwrapv")
endif()
```

## 診斷步驟

### 1. 啟用詳細日誌

在 launch 文件中添加：

```xml
<env name="ROSCONSOLE_CONFIG_FILE" 
     value="$(find sbpl_lattice_planner)/config/debug.conf"/>
```

創建 `config/debug.conf`：
```
log4j.logger.ros.sbpl_lattice_planner=DEBUG
```

### 2. 測試不同象限的座標轉換

創建測試腳本 `test_quadrants.cpp` 或使用 ROS 服務測試：

```bash
# 測試第一象限 (x > 0, y > 0)
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped ...

# 測試第二象限 (x < 0, y > 0)
# 測試第三象限 (x < 0, y < 0) - 已知可用
# 測試第四象限 (x > 0, y < 0)
```

### 3. 檢查 SBPL 內部狀態

在 `sbpl_lattice_planner.cpp` 中添加調試輸出：

```cpp
ROS_DEBUG("[SBPL] SetStart returned state ID: %d", ret);
ROS_DEBUG("[SBPL] SetGoal returned state ID: %d", ret);
```

如果返回負數，表示座標轉換失敗。

## 參考資料

- SBPL GitHub Issue: https://github.com/sbpl/sbpl/pull/27
- ROS Navigation Experimental: https://github.com/ros-planning/navigation_experimental/issues/57
- ARM 架構整數運算: https://gcc.gnu.org/onlinedocs/gcc/Integers-implementation.html

## 實施建議

1. **優先實施方案 A**：使用 `worldToMap` 確保座標轉換正確
2. **添加方案 C 的編譯選項**：提高 ARM 兼容性
3. **添加詳細日誌**：診斷具體失敗位置
4. **測試驗證**：在不同象限測試規劃功能




