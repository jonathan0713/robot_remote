# robot_remote → OpenArm（模擬）改造計畫

狀態：草案，2026-10-21，Claude。分支 `feat/openarm-sim`。這份計畫只涵蓋模擬，實機後端（igus CRI / igus ROS driver）不動。

## 1. 目標與範圍

- 用 Meta Quest 3 遙控 **OpenArm 雙臂（每臂 7 軸 + 平行夾爪）**，在 ITRI_project 的 **Isaac Sim 5.1** `held_tote_pick` 場景（`isaac_save/main/Oct/1015.usd`）收人工示範。
- 收到的 Episode 必須和現有 `held-tote-pick-v5-lerobot` **同格式**，可以直接和 v5、DAgger、DART 資料混用訓練：LeRobot v2.1、16 維 state/action、3 支相機各 224×224、30 fps、`demonstration_source = "quest3"`。
- 不在範圍內：OpenArm 實機、頭盔內顯示 Isaac 畫面（操作者看螢幕上的 Isaac 視窗），以及 robot_remote 自己的 ReBeL 場景改寫。

## 2. 核心設計決定（建議）

**robot_remote 只負責「Quest → 關節命令」這一層；Isaac 場景、錄製、匯入沿用 ITRI_project 現有的 chain flow。**

理由：

| | 走 robot_remote 自己的 sim + recorder | 走 ITRI chain flow（建議） |
|---|---|---|
| Isaac 場景 | 要把 `run_dual_arm_sim.py` 改成 OpenArm 1015 場景，重做 bridge、相機、reset | `run_tote_pick_scene.py` 已有 `/joint_command` 訂閱、`/isaac_joint_states`、相機 |
| 左臂拉箱 | 要自己做 | `tote_left_ros_teacher.py` 已驗證（profile B） |
| 資料格式 | LeRobot v3.0（lerobot 0.5.1），和 v5 不相容 | 同一個 recorder 和 importer，格式一致 |
| Isaac 版本問題 | recorder 綁 Isaac 6.1 rclpy / py3.12，要改 | 不需要 |

ITRI 的 chain flow 已經有一個專給外部右臂控制器用的接口：`TOTE_CHAIN_ASSIST=1`（`scripts/vla/run_tote_chain_flow.sh:53-58, 258-312`）。

- 左臂 teacher 拉出周轉箱並夾住，之後 pick stack 不會啟動。
- 開放 `<run_dir>/capture.fifo`：寫一個 byte 開始錄製，再寫一個 byte 結束（`rollout_held_tote_controller.py:467`）。
- 右臂命令送到 `/right_joint_command`，經 `joint_command_mux.py` 合併到 `/joint_command`。
- 結束時 flow 依 `<run_dir>/measurements.json` 是否存在判斷 pick 成敗；touch `<run_dir>/release` 會結束持箱。

Quest teleop 就直接取代現在 `rollout_held_tote_controller.py --expert ik` 在這個接口裡的角色。

**程式碼分工**

- **robot_remote**：通用的 Quest teleop，加上 robot profile。ReBeL 和 OpenArm 各是一個 profile，讓 upstream 的修正仍然能合併進來。
- **ITRI_project**：只放 held-tote 專用的 glue，例如一個 `scripts/vla/run_held_tote_teleop.py`，用 ASSIST + RECORD + `--gui` 啟動 chain 和 teleop，並提供操作者的按鍵流程。

## 3. 需要改的 robot_remote 部件

### 3.1 Robot profile（新增 `config/robots/*.json`）

把現在散在各檔案的 ReBeL 常數集中到 profile（各常數的位置見附錄 A）。OpenArm profile 的內容：

| 欄位 | OpenArm 值 | 來源 |
|---|---|---|
| IK URDF | `ITRI_project/ros2_ws/src/openarm_dual_gripper_description/urdf/openarm_dual_gripper.urdf`（從 Isaac USD 匯出，與 MoveIt 相同） | 路徑由環境變數或設定提供 |
| chain base / tip | `openarm_body_link` → `openarm_{left,right}_ee_tcp`（link7 + 0.1931 m） | 同上 URDF |
| 手臂關節 | `openarm_{side}_joint1..7` | Isaac articulation |
| 夾爪 | `openarm_{side}_finger_joint1`（prismatic，0 = 閉，0.040 = 開，硬限 0.044；joint2 是 mimic） | `docs/validation/gripper-opening.md` |
| 底座在 world | robot root (0, 0, 0.45)，無旋轉；手臂底座已在 URDF 內 | 1015.usd |
| 狀態 / 命令 topic | 讀 `/isaac_joint_states`（22 DOF）；寫 `/right_joint_command`、`/left_joint_command`（JointState 子集，經 mux） | chain flow |
| home | 從 `config/tote_pick/held_tote_start_v5.json` 的 right ready pose 讀取 | ITRI 設定 |
| 速度上限 | 先用 0.5 rad/s、0.25 m/s、60°/s（低於 ReBeL 的 sim 預設），之後依手感調整 | 待調 |
| reach 限制 | 由 FK 量測 OpenArm 工作空間後決定（ReBeL 是 0.55 m） | P1 |
| 控制器 → 工具軸 | 每臂各一組。左臂 link7 繞 X 轉了 180°，左右不能共用同一組 | P1 量測 |
| robot_type | `openarm_dual_gripper` | 與 v5 一致 |

### 3.2 IK：`SerialChain` 改成 n 軸，並處理冗餘自由度

現有的解法是 task-priority DLS：位置是主任務，姿態放在位置的零空間（`quest_arm_teleop.py:83-175`）。演算法本身通用，但有下面這些要改：

1. **寫死 6 軸**：`np.full(6)`（99 行）、`np.zeros((6, 6))`（142 行）、`np.ones(6)`（161 行）、`np.eye(6)`（166 行），全部改成 `n = len(joint_names)`。
2. **零行程 revolute**：匯出的 URDF 用零行程 revolute 表示 `openarm_{side}_hand` 和 `ee_tcp_joint`。現在的程式會在 `joint_names.index()` 丟例外。改成：不在 `joint_names` 裡的關節一律當 fixed（角度 0）。
3. **第 7 軸的冗餘**：7 軸時零空間在位置和姿態都滿足後還剩 1 維（elbow self-motion）。最小範數 DLS 會讓手肘隨 seed 漂移。加一個第三優先的 posture 任務，把手臂拉回 home／ready 姿態並遠離關節限位，權重要低。OpenArm j6 只有 ±45°，j4 是 0..140°，這兩軸特別需要避開限位。
4. **「肩膀」的定義**：現在用 `joint_origin(1)` 當 reach 的球心，只在 joint1 的軸穿過 joint2 原點時成立。改成 profile 指定的 link，或固定用 joint2 在 base 中的位置，並在 P1 用 FK 驗證。
5. **驗證方式**：用 TurboVLA 的 `experiments/openarm/motion/arm_kinematics.py`（`ArmKinematics`，已在 held-tote 驗證過）當參考，比對隨機關節角下 `ee_tcp` 的 FK，誤差要小於 1e-6 m。robot_remote 不 import TurboVLA；測試只在找得到 TurboVLA 時才跑。

替代方案：直接用 TurboVLA 的 `RateLimitedMagneticServo` / `bounded_dls.py`。缺點是 robot_remote 會綁上 TurboVLA 的 import 和 venv（可編輯安裝陷阱）。建議先維持 robot_remote 自帶 IK。如果 P1 的追蹤品質不夠，再評估換掉。

### 3.3 Teleop 主程式（`quest_dual_arm_teleop.py`）

- 新增 `--robot openarm` 和 backend `isaac-openarm`：分臂發布到 profile 指定的 topic，關節名稱和夾爪對應全部由 profile 決定。
- **只驅動選定的手臂**（P2 只有右臂）。未選的手臂不發布任何命令，否則會和左臂 teacher 搶 mux。
- 夾爪：`finger_joint1 = open − (open − closed) · trigger`，open = 0.040，closed = 0.0。在 sim 中用位置驅動夾住物體即可（stiffness 1e4、maxForce 60）。
- 按鍵在 held-tote 模式下的意義：
  - **A**：開始／結束 capture，寫到 `--capture-fifo`。
  - **B**：放棄這個 attempt，寫 `<run_dir>/teleop_discard.json`，importer 會略過它。
  - **Y**：結束 attempt（touch `<run_dir>/release`）。
  - 舊的 `/quest_teleop/events` 保留。
- 結束時寫 `<run_dir>/measurements.json`，欄位與 `held_tote_evidence.measurements()` 相同：`object_clearance_above_rim_m`、`stable_hold_s`、`pipeline_success`、`pipeline_phase = "teleop"`。數值從 `/tf` 的物件和箱緣讀取。可以沿用 `rollout_held_tote_controller.py` 的計算方式，P2 再確認細節。
- 兩臂碰撞 guard：`arm_collision.py` 寫死 6 個關節原點加上 ReBeL 的工具點（32-53 行）。改成依 profile 的 link 清單建立膠囊。P2 只動右臂時，左臂位置用 `/isaac_joint_states` 的量測值，因此 guard 從第一天就有效。
- `make_urdf()`（xacro igus）改成從 profile 讀 URDF 檔案。

### 3.4 不需要改的部分

`QuestController`（isaacteleop / CloudXR）、`OPERATOR_T_ANCHOR`、clutch、增量映射、`tool_from_controller`、Cartesian 和關節速度限制、anti-windup、`LoopStats`、stdin 指令、rot-frame 切換、`teleop_viz` 的 lag 統計、launcher 的程序管理。

### 3.5 暫不處理

`dual_arm_sim/*`（ReBeL 場景）、`recording/*`（LeRobot v3 recorder）、`ethernet_rebel.py`、`real_robot/*`。程式碼保留，ReBeL profile 繼續用它們。

## 4. 環境與執行設定

- **Quest 環境**：`setup/install_quest_env.sh` 會建立 `.venv_quest`（py3.10、system site packages、`isaacteleop[cloudxr]==1.3.131`、`numpy<2`）。CloudXR EULA 由使用者本人接受一次。這和 Isaac 版本無關，Isaac 5.1 不影響。
  - **需要使用者同意才安裝**（專案規則）。
- **ROS 設定必須和 chain 一致**：`ROS_DOMAIN_ID=42`、`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`、`FASTRTPS_DEFAULT_PROFILES_FILE=.../tote_collection_fastdds_large_shm.xml`。chain **沒有**設 `ROS_LOCALHOST_ONLY`，但 robot_remote 的 `ros_env.sh` 設成 1。混用可能導致 discovery 失敗，所以 OpenArm 模式要用 chain 的設定，不 source `ros_env.sh`。
- **模擬速度**：1015 場景約 1.3 s 牆鐘時間才走 1 s 模擬時間，`--gui` 可能更慢。teleop 以牆鐘 30 Hz 送位置目標，recorder 以 sim time 30 Hz 取樣，所以 action 標籤沒問題。操作者會感覺到延遲，P2 要量化這個延遲。
- **重置**：每個 attempt 都會冷啟動 Isaac，等待時間要在 P2 實測。如果太長，可以評估 `claude/isaac-persistent-reset-2f299c` 分支的 persistent Isaac。
- **大量 IO** 一律寫到 DATA 磁碟（`/media/jonathan/DATA/VLA_data/held-tote-pick/<campaign>/attempts/*`）。DATA 未掛載時要停下來詢問，不能寫到本機磁碟。

## 5. 分階段計畫

每一階段都要先過驗收才進下一階段。標 ⚠ 的步驟需要使用者明確同意（安裝、啟動 Isaac 或 ROS）。

**P0：環境和 Quest 連線（不碰機器人）**
- ⚠ 安裝 `.venv_quest`；使用者接受 CloudXR EULA。
- ⚠ `quest_probe.py`：Quest 瀏覽器連上 CloudXR，終端機印出兩個控制器的 pose、squeeze、trigger、按鍵。
- 驗收：10 Hz 以上穩定的 tracking，按鍵邊緣都能偵測到。如果連不上，查防火牆和 Wi-Fi。

**P1：離線運動學（不需要 Isaac 或 ROS）**
- 建立 robot profile 結構；ReBeL profile 必須保持原有行為。
- `SerialChain` 改成 n 軸，處理零行程關節，加入 posture 任務。
- 測試：
  1. 與 TurboVLA `ArmKinematics` 比對 FK。
  2. 在 held-tote 右臂工作空間（ready → 箱內 → 出箱）取樣目標，IK 收斂率和每 tick 的最大關節步長。
  3. 7 軸的手肘在固定 TCP 下不漂移。
  4. 假 Quest 加假 Isaac 的整合測試：握持、放開、再握持不跳動；topic 只有右臂關節名稱。
- 量測左右臂 tool 軸方向，決定 `tool_from_controller` 的參數。
- 驗收：上述測試全部通過；ReBeL 既有測試在有 igus_ws 時不退步。沒有 igus_ws 時會 skip，要說明。

**P2：模擬中只遙控右臂（ASSIST 模式）**
- ITRI 端新增 `scripts/vla/run_held_tote_teleop.py`：以 `TOTE_CHAIN_ASSIST=1 TOTE_CHAIN_RECORD=1`、`--gui` 啟動一個 attempt；等到「tote pulled and held」後啟動 teleop（`--capture-fifo`、`--run-dir`）；結束後收尾。一次只跑一個 Isaac。
- ⚠ 先不錄製試跑：手感、延遲、夾爪、guard、Y 結束。
- ⚠ 再錄 3–5 個 soda can 的 attempt → 用現有 importer 轉成 v2.1 → 檢查 16 維對齊、影像、`demonstration_source`、QA 結果。
- 驗收：
  - 匯入的 Episode 能被訓練 recipe 讀取。
  - 開環 replay 和 `/isaac_joint_states` 的誤差在 v5 的水準內。
  - 單一 attempt 的週期時間（含冷啟動）有實測數據。

**P3：資料收集流程**
- 依用途決定收集方式（見第 6 節問題 1）：
  - 完整人工示範：多個物件和擺放位置，搭配 campaign config。
  - 針對 soda can 側向精度的人工修正片段：與 DAgger／DART 結果比較。
- 操作者手冊：在 robot_remote 的 `docs/` 加一頁 OpenArm 說明。

**P4（選做）：兩臂都由人遙控**
- chain 新增模式：不啟動左臂 teacher，左臂也由 Quest 控制。
- 拉箱要用角落側緣夾持，周轉箱重心不能離開層板，持箱期間左夾爪需要 2 mm 的 squeeze（`--pull-squeeze-m 0.002`）。用 trigger 的線性映射很難穩定維持，可能需要一個「鎖定夾持」按鍵。
- Episode 邊界和左夾爪 action slot 的語意由 TurboVLA ADR-0006 決定，需要先確認。

## 6. 待使用者決定

1. **人工資料的用途**：完整示範（新的 demonstration source），還是 soda can 這類困難案例的修正資料？這會影響 P3 的設計和錄製邊界。
2. **ReBeL 支援**：要不要繼續在這個 fork 裡維護（profile 化），還是直接刪掉、只留 OpenArm？profile 化成本較高，但可以持續合併 upstream 的修正。
3. **操作者視角**：看螢幕上的 Isaac 視窗是否可以接受？如果要在頭盔內看到 Isaac 畫面，需要 Isaac 的 XR 串流，這是另一個工作量。
4. **P4 的時程**：要不要排入？

## 附錄 A：ReBeL 寫死的位置（改造檢查清單）

- `quest_teleop/quest_arm_teleop.py`：`ARM_JOINTS` 38、`HOME_DEG` 42、`GRIPPER_TCP_IN_LINK6` 47、`TCP_PRESETS` 50-54、6 軸陣列 99/142/161/166、`base_link`/`link6` 316/336、gripper limits 324-325、reach 0.55 329/337、CloudXR app 名稱 208。
- `quest_teleop/quest_dual_arm_teleop.py`：`ARM_BASE_XYZ`/`YAW` 58-59、速度上限 62-63、`make_urdf()` 156-161、關節／夾爪名稱 169-170/291-294/330/448、gripper 範圍 565-566、`--park-deg`/`--other-arm-deg` nargs=6 513/559、chain base/tip 599。
- `quest_teleop/arm_collision.py`：6 個關節原點加 ReBeL 工具點 32-53。
- `recording/record_dataset.py`、`replay_episode.py`：14 維 layout、`robot_type`、slice `v[i:i+6]`。（P2 不使用這兩個檔案。）
- `run_record.sh`、`run_replay.sh`、`setup/check_env.sh`、`install_record_env.sh`：寫死 Isaac 6.1 的 `isaacsim.ros2.core` 和 py3.12。（P2 不使用。）
