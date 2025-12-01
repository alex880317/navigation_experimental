# SBPL Lattice Planner ARM 架構問題深度分析

## 問題確認

根據您的觀察：
- **第三象限（車子座標系，x<0, y<0）**：可以成功規劃 ✅
- **其他象限**：無法規劃 ❌
- **關鍵差異**：第三象限需要**後退**（x 減小），其他象限需要**前進**（x 增加）

## ARM vs x86 架構差異分析

### 1. 整數運算差異

ARM 和 x86 在以下方面可能有差異：

#### 1.1 負數的取模運算 (`%`)
```cpp
// x86: -5 % 3 = -2 或 1（取決於實現）
// ARM: 可能不同
int result = negative_coord % positive_value;
```

#### 1.2 整數溢出行為
```cpp
// 如果使用 unsigned int 處理負數
unsigned int x = -1;  // 在 ARM 上可能變成很大的正數
```

#### 1.3 狀態空間索引計算

SBPL 內部可能使用類似這樣的計算：
```cpp
// 假設的狀態 ID 計算（SBPL 內部）
int state_id = (x_cell * height + y_cell) * num_angles + angle_index;

// 如果 x_cell 或 y_cell 是負數，在 ARM 上可能：
// 1. 溢出變成很大的正數
// 2. 取模運算結果不同
// 3. 導致狀態 ID 計算錯誤
```

### 2. 座標轉換問題

從日誌看：
- Start rel=(27.13, 5.50) → state ID: 329
- Goal rel=(25.79, 5.75) → state ID: 2

**關鍵觀察**：
- Start x (27.13) > Goal x (25.79) → **需要後退**
- State ID 329 到 State ID 2 的啟發式值可能是異常的

### 3. 可能的根本原因

#### 3.1 SBPL 環境初始化問題

SBPL 環境 (`EnvNAVXYTHETALAT`) 在初始化時可能：
1. 假設座標從 (0,0) 開始
2. 使用 `unsigned int` 存儲座標索引
3. 負座標被錯誤解釋為很大的正數

#### 3.2 Motion Primitives 應用問題

Motion Primitives 在應用時：
1. 可能只支持前進方向（x 增加）
2. 後退方向（x 減少）的 primitives 可能未正確生成
3. 或者後退方向的狀態轉換在 ARM 上計算錯誤

#### 3.3 啟發式函數計算問題

啟發式函數 (`GetFromToHeuristic`) 可能：
1. 使用歐幾里得距離計算
2. 在 ARM 上，負座標的距離計算可能溢出
3. 返回異常大的值，導致規劃器認為不可達

## 診斷步驟

### 步驟 1: 檢查啟發式值

已添加的日誌會顯示：
```
Heuristic value from start(X) to goal(Y): Z
```

如果 Z 是異常大的值（≥1000000），說明問題在啟發式計算。

### 步驟 2: 檢查狀態空間範圍

需要檢查 SBPL 環境的狀態空間：
- 最小/最大 x 座標
- 最小/最大 y 座標
- 狀態總數

### 步驟 3: 檢查 Motion Primitives

檢查 motion primitives 是否包含後退動作：
```bash
grep "endpose_c: -" mycar_jetson.mprim
```

如果沒有負 x 的 endpose，說明 primitives 不支持後退。

## 可能的解決方案

### 方案 1: 修復座標轉換（如果問題在座標轉換）

確保傳給 SBPL 的座標始終為正：
```cpp
// 將負座標轉換為相對於地圖原點的絕對座標
double start_x_rel = start.pose.position.x - costmap_ros_->getCostmap()->getOriginX();
// 如果 start_x_rel < 0，可能需要特殊處理
```

### 方案 2: 檢查 Motion Primitives（如果問題在 primitives）

確保 primitives 包含後退動作：
- 檢查 `mycar_jetson.mprim` 中是否有 `endpose_c: -1 0 0` 或類似的後退動作
- 如果沒有，需要重新生成包含後退的 primitives

### 方案 3: 修復 SBPL 環境初始化（如果問題在 SBPL 內部）

這需要修改 SBPL 庫本身，可能需要：
1. 檢查 SBPL 源碼中的狀態索引計算
2. 確保負座標被正確處理
3. 可能需要使用 `int` 而不是 `unsigned int` 存儲座標

## 下一步行動

1. **重新編譯並運行**，查看啟發式值
2. **檢查 motion primitives**，確認是否包含後退動作
3. **對比成功和失敗的情況**，找出關鍵差異
4. **如果啟發式值異常**，可能需要修改 SBPL 環境的座標處理邏輯

## 參考資料

- SBPL GitHub: https://github.com/sbpl/sbpl
- ARM 整數運算: https://gcc.gnu.org/onlinedocs/gcc/Integers-implementation.html
- ROS Navigation Experimental: https://github.com/ros-planning/navigation_experimental





