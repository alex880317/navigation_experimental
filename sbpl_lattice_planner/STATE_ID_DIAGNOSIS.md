# State ID 異常問題診斷

## 問題觀察

從日誌中發現：
- Start rel=(27.126, 5.46684) → **State ID: 1** ⚠️
- Goal rel=(25.6371, 5.77975) → **State ID: 2** ⚠️

## 異常分析

### 正常情況下的 State ID 計算

如果地圖大小是 682x344，角度離散化為 16，那麼狀態 ID 應該是：
```
state_id = (x_cell * height + y_cell) * num_angles + angle_index
```

對於 Start: map=(542, 109)，假設 angle_index=0：
```
state_id = (542 * 344 + 109) * 16 + 0
         = (186448 + 109) * 16
         = 186557 * 16
         = 2,984,912
```

但實際返回的是 **1**，這表明：

1. **SBPL 可能使用不同的座標系統**
   - SetStart/SetGoal 可能期望的是 map 座標（0-based 索引）
   - 而不是相對座標（相對於 origin 的米制座標）

2. **或者 SBPL 環境的初始化有問題**
   - 環境可能沒有正確初始化狀態空間
   - 或者座標轉換在 ARM 上有 bug

## 可能的解決方案

### 方案 1: 使用 Map 座標而不是相對座標

嘗試使用 map 座標（cell 索引）：
```cpp
// 使用 map 座標轉換為米
double start_x_map = start_mx * costmap_resolution;
double start_y_map = start_my * costmap_resolution;
double goal_x_map = goal_mx * costmap_resolution;
double goal_y_map = goal_my * costmap_resolution;

int start_state = env_->SetStart(start_x_map, start_y_map, theta_start);
int goal_state = env_->SetGoal(goal_x_map, goal_y_map, theta_goal);
```

### 方案 2: 檢查 SBPL 環境的座標系統

SBPL 環境可能在初始化時設定了座標原點，需要確認：
- 環境的原點位置
- 座標系統的方向
- 是否支持負座標

### 方案 3: ARM 架構特定的修復

如果問題確實在 ARM 架構上，可能需要：
1. 檢查 SBPL 源碼中的狀態索引計算
2. 確保整數運算的一致性
3. 可能需要修改 SBPL 庫本身

## 下一步

1. 檢查 GetCoordFromState 返回的實際座標
2. 對比成功和失敗情況的 State ID
3. 如果 State ID 異常小，嘗試使用 map 座標




