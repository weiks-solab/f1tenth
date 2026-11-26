# 實車超車節點 · ROS 2 Humble

這個分支可單獨放入車上電腦建置。五個節點使用即時 ROS 訊息、系統時鐘與車輛 TF。啟動後可從候選路徑一路觀察到 Ackermann 指令。預設指令流向演算法輸出 `/overtaking/drive`。隨附的兩張地圖仍屬 Stage B，幾何資料保留 `planning_allowed=false`。此版本可用於車上接線、運算與輪台前的檢查，實際行駛仍需完成地圖、車身模型、轉向方向及底盤仲裁驗收。

## 建置與啟動

在這份 repo 的根目錄執行。已有 ROS 2 Humble、colcon 並初始化 rosdep 的車上電腦可使用下列命令。

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/f1tenth_overtaking_core src/overtaking_msgs src/car_control src/overtaking_ros2 --ignore-src -r -y
colcon build --packages-up-to overtaking_ros2
source install/setup.bash
ros2 launch overtaking_ros2 vehicle.launch.py
```

預設載入 `my_map`。另一張圖可用 `map_id:=new`。所有路徑由已安裝的 package share 解析。節點可以先啟動等待訊息，預設輸入名稱是本分支約定的接線位置。用 `ros2 launch overtaking_ros2 vehicle.launch.py --show-args` 查看可調整的 topic、車體 frame、地圖與模型路徑。已有其他節點提供相同格式的 `/overtaking/relative_state` 時，加上 `start_relative_state:=false`，由該發布者提供狀態。

## 五個節點的資料流

下列短名稱均位於 `/overtaking`。地圖公告使用 reliable、transient local、depth 1，後啟動的節點也能讀到。規劃 topic 使用 reliable、depth 1，里程計與轉向輸入採用 sensor-data QoS。

| 節點 | 讀取或訂閱 | 任務與發布 | 尚需車上確認 |
|---|---|---|---|
| `map_assets` | 安裝好的地圖資產與固定 SHA-256 | 啟動時驗證檔案並發布 `map_assets` 的地圖、幾何與安全域識別碼 | 定位使用的地圖、原點與方向一致，完成實際賽道驗證 |
| `relative_state` | 兩車 `Odometry`、自車 `SteeringState` | 對齊量測時間，投影至 Frenet 座標，發布 `relative_state`，包含間距、相對速度、量測有效性與原因 | 自車與對車的實際發布者、轉向與側向速度來源，見下節 |
| `path_generation` | `map_assets`、`relative_state`、`planning_policy` | 對同一筆狀態生成五個候選槽位，依序檢查安全域、曲率與三圓車身，發布 `candidate_paths` 與各槽位的有效性及原因 | 量測車上延遲與車身尺寸 |
| `overtaking_decision` | `map_assets`、`relative_state`、`candidate_paths` | 發布 `planning_policy` 指定中心線或回正策略，使用原有單軌預測、車對車及車對牆碰撞檢查，決定巡航、超車、跟車、回正或停止。發布 `target_path` | 校正模型及轉向延遲，量測截止時間內成功選路比例 |
| `pure_pursuit` | `target_path`、TF 的 `map → ego_body_frame` | 沿所選點列取前視點，發布 `/overtaking/drive` 與 `/pure_pursuit/target_marker` | 輪台核對正負號、轉向上限、底盤仲裁與急停 |

```mermaid
flowchart LR
  O[兩車里程計與自車轉向回授] --> R[relative_state]
  M[map_assets] --> G[path_generation]
  M --> D[overtaking_decision]
  R --> G
  R --> D
  D -->|planning_policy| G
  G -->|candidate_paths| D
  D -->|target_path 點列與速度| P[pure_pursuit]
  TF[自車 TF] --> P
  P -->|AckermannDriveStamped| C[/overtaking/drive]
```

## relative_state 的訊息格式與待確認接線

| 預設 topic | 型別與必要欄位 | 需與車上團隊確認 |
|---|---|---|
| `/overtaking/input/ego_odom` | `nav_msgs/msg/Odometry`。pose 在 `map`，child 預設 `base_link`。twist 在車體座標，`linear.x/y` 為前向／側向速度 m/s，`angular.z` 為偏航角速度 rad/s | 定位與速度的時間對齊、車體參考點與 TF 一致。VESC 若只量到前向速度，側速須由估測器提供。不可把未量測欄位的預設零值當成有效量測 |
| `/overtaking/input/opponent_odom` | 同型別，pose 在同一 `map`，child 預設 `opponent/base_link`，提供前向速度 | 對車定位來源、車體參考點、時鐘同步、遺失偵測 |
| `/overtaking/input/steering` | `overtaking_msgs/msg/SteeringState`。ego 車體 frame、量測時間、實際前輪角、仲裁後保持的指令、最近兩個 10 ms 模型步的延遲輸入（新到舊），以及 50 ms 控制週期內的相位 0–4 | 轉角回授／估測、指令歷史、相位與實際延遲。所有角度使用 rad、左轉為正，須在同一時基對齊 |
| `/overtaking/relative_state` | `overtaking_msgs/msg/RelativeState`。包含 state ID、geometry ID、兩車時間戳、Frenet 狀態、對車狀態與 `rollout_valid` | 若由別人完成整合，可直接發布此格式，且只有一個發布者。可用 `ros2 interface show overtaking_msgs/msg/RelativeState` 看完整欄位 |

內建節點以 20 Hz 檢查最新量測，接受齡期 150 ms 以內、兩車時間差 30 ms 以內的新資料對。初次定位假設兩車相隔小於半圈，再以展開的弧長跨圈追蹤。`delta_s_unwrapped_m` 正值代表對車在前。兩車定位齊全時即可看候選路徑。轉向回授缺少、過期或未對齊時，決策輸出 `STOP`。遇到定位跳躍造成 `reinitialization_required`，先確認兩車位置，再重新啟動以建立新追蹤序列。

## 地圖、選路與更新頻率

- `prepare_map_assets` 是離線工具。隨附中線、Frenet 安全域、5 mm XY 三圓中心查表與原始 PGM/YAML。執行時各節點從同一版本讀檔。在不確定的格點仍執行原有精確碰撞檢查。資產可透過 `ros2 run overtaking_ros2 prepare_map_assets --help` 重新建立。新的資產需以工具輸出的 manifest SHA-256 傳入 launch。
- `my_map` 的 Frenet 網格為 1 mm、規劃長 3 m。`new` 的 Frenet 網格為 5 mm、規劃長 1.45 m。有效候選沿參考弧長約每 2 cm 取一點。決策發布整條局部點列及 `target_speed_mps`，由既有 PP 選取前視點。
- 決策使用 100 ms 的軟體計時器。每次發布上一輪已完成且仍有效的結果，並對最新狀態提出下一輪請求。候選與決策用相同的 state ID 和期限配對。無法在本輪完成、資料失效或沒有可行軌跡時，發布具原因的零速 `STOP`。無法超車時，既有選路器會先檢查安全跟車方案。運算在背景執行，逾期結果捨棄。Python、DDS 與 Linux 的排程仍有抖動，實車需分別量測發布率和有效規劃比例。
- PP 控制週期 50 ms，追蹤有效路徑最長至來源時間加 250 ms，TF 齡期上限 150 ms。收到 `STOP`、軌跡過期或 TF 無效便輸出零速。直接啟動原本的 CSV 模式仍使用原本的入口與設定。
- 預測步長為 10 ms。原有動力模型與三圓檢查保留，模型轉向更新對齊 PP 的 50 ms 控制週期。`config/vehicle_model.yaml` 提供初始參數，規劃曲率上限、預測器及 PP 共用轉角限制與軸距。隨附初值為 0.42 rad、0.33 m，轉向比例與符號仍須實測。車身 0.51 × 0.27 m 若有變更，應一併重建地圖查表。

## 車上觀察

```bash
ros2 node list
ros2 topic hz /overtaking/target_path
ros2 topic echo /overtaking/relative_state --field reason
ros2 topic echo /overtaking/candidate_paths --once
ros2 topic echo /overtaking/target_path --field reason
ros2 topic echo /overtaking/drive --field drive
```

可比對 `state_id`、`plan_id`、`candidate_id`、點列、目標速度以及各候選的 `rejection_reason`。找到 `no_timely_plan` 表示該輪沒有及時可用的結果。底盤整合點是 `/overtaking/drive → 車上的仲裁、急停與命令逾時保護 → /drive → VESC bridge`。這份 launch 啟動到演算法輸出為止，定位、TF 和底盤各自由車上原本的 bringup 負責。

## 檔案安排與測試

| 路徑 | 用途 |
|---|---|
| `src/overtaking_msgs/msg/` | 對外訊息與欄位定義 |
| `src/f1tenth_overtaking_core/` | 原本的幾何、生成、選路、動力預測與追蹤演算法，以及上游來源與對照測試 |
| `src/overtaking_ros2/` | 四個 ROS 節點、離線資產工具、兩份地圖、車輛參數與統一 launch |
| `src/car_control/` | 原 repo 的 Pure Pursuit 加上局部軌跡訂閱模式 |

在獨立的 Humble 測試電腦上，從 repo 根目錄載入剛建置的 overlay 後可執行下列整合測試。它會自行啟動五個節點，透過 ROS 即時提供測試量測，檢查候選、決策、PP 和失聯停車。請使用獨立的 ROS domain。

```bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=76 python3 src/overtaking_ros2/test/check_vehicle_launch.py
```
