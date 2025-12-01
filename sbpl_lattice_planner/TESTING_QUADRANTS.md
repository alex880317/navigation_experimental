# SBPL Lattice Planner 象限測試指南

## 測試不同象限的規劃功能

### 測試步驟

1. **啟動 move_base**
```bash
roslaunch mycar_start move_base.launch
```

2. **啟用調試日誌**
```bash
# 在另一個終端
rosrun rqt_logger_level rqt_logger_level
# 將 ros.sbpl_lattice_planner 設置為 DEBUG 級別
```

3. **測試不同象限**

#### 第一象限 (x > 0, y > 0)
```bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
  "header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: 'map'
  pose:
    position:
      x: 2.0
      y: 2.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0" -1
```

#### 第二象限 (x < 0, y > 0)
```bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
  "header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: 'map'
  pose:
    position:
      x: -2.0
      y: 2.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0" -1
```

#### 第三象限 (x < 0, y < 0) - 已知可用
```bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
  "header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: 'map'
  pose:
    position:
      x: -2.0
      y: -2.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0" -1
```

#### 第四象限 (x > 0, y < 0)
```bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
  "header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: 'map'
  pose:
    position:
      x: 2.0
      y: -2.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0" -1
```

### 觀察日誌輸出

檢查以下調試信息：
```
[sbpl_lattice_planner] Start: world=(...) map=(...) rel=(...) theta=(...)
[sbpl_lattice_planner] Goal: world=(...) map=(...) rel=(...) theta=(...)
[sbpl_lattice_planner] SetStart returned state ID: ...
[sbpl_lattice_planner] SetGoal returned state ID: ...
```

### 預期結果

修復後，所有象限都應該：
1. 成功設置 start 和 goal（state ID >= 0）
2. 成功規劃路徑
3. 在 RViz 中顯示路徑

### 如果問題仍然存在

1. **檢查座標轉換數值**：比較不同象限的 `rel=(...)` 值
2. **檢查 state ID**：如果返回負數，表示 SBPL 內部座標轉換失敗
3. **檢查 costmap origin**：確認 origin 位置是否正確
4. **檢查 motion primitives**：確認在 Jetson 上生成的 primitives 是否正確





