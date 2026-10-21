# robot_remote — Codex / Claude 共用工作規則

## 唯一共用入口

- Codex 與 Claude 每次開始工作都先讀本檔、`README.md` 與 `git status --short`。
- 本資料夾的需求、決策、目前狀態、驗證證據與待辦，更新在本檔；不要只留在聊天、個別代理的私有記憶或 `/tmp`。
- 原始碼、設定、測試、文件、日誌及新資料集都放在本專案內。日誌在 `logs/`，設定在 `config/`，新資料集預設在 `datasets/`。不要任意搬移既有資料集或大型環境。
- 外部既有依賴保留原處；使用時在本檔或 README 註明實際位置與用途。`.venv_quest/`、`igus_ws/` 為現有本地環境。
- 既有外部執行環境：`/opt/ros/humble`（ROS Humble）、`/home/hucenrotia/.cloudxr`（CloudXR 既有 runtime/cache）；資料錄製另依 README / `recording/run_record.sh` 的 LeRobot 環境。這些是依賴，不是交接資料存放處。
- `CLAUDE.md` 連結至本檔，讓兩個工具讀到同一份規則，不維護兩份互相矛盾的交接文件。
- 同時工作前先查看下方進度及檔案變更；不要覆蓋其他代理或使用者未提交的修改。大範圍修改先在此登記負責檔案，交接時更新完成項目、測試、限制及下一步。
- 未經使用者要求，不自行新增分身代理或啟動其他聊天；共用檔案不代表可以同時操作同一台機器人。

## 實體機器人

- 左臂 `192.168.3.11`、右臂 `192.168.3.12`。CRI 控制 / 狀態 TCP 預設 3920；Position Interface 埠必須向 CRI 查詢，不硬猜。
- 預設實體路徑（2026-10-16 使用者決定）為 igus ROS driver（`real_robot/dual_arm_driver.launch.py`，/left、/right）+ teleop `--backend real-ros` 以 ROS topic 送軌跡命令，與 Isaac 模擬相同做法。Ethernet CRI + Position Interface 直連保留為實驗選項 `--backend real`。
- 實體驗證先用 `--check-only` 或 `--dry-run`。禁止把「能連線／模擬測試通過」寫成「實體動作已驗證」。
- 本次開發允許讀取兩臂狀態，未執行任何實體運動、馬達啟用、歸零或參考點搜尋。後續運動測試必須由現場操作者看守急停並明確啟動。
- （已更新，見文末「實機改走 igus ROS driver」：現在連上後先回 home、結束回全軸 0°；driver 連線時自行啟用馬達。）Ethernet 直連版：馬達啟用只在使用者選擇 `--enable-motors`（GUI「Enable motors at start」預設勾選，2026-10-16 使用者要求）時，且只在唯一問題是 MNE 時送 Reset + Enable、不搶其他 CRI 客戶端的控制權、不因斷線自行恢復運動。單臂故障須讓本次選用的兩臂停止串流。
- 使用者決定（2026-10-16）：正常結束（Stop all / Ctrl+C）前，實體手臂以 10°/s 自動回到全軸 0°（`--park-deg`）；再按一次即停。dry run、故障（斷線、回授逾時、急停等）或 `--no-park` 時不回位。
- 模擬中的底座位姿與工具幾何尚非實體校正結果；點距離 guard 不等同完整碰撞檢查，不能作為人員安全保護。

## 目前工作與交接（2026-10-15，Codex）

- 需求：將 Claude 的單臂 ROS topic 實體版本改成雙臂 Ethernet 直連，保留 Quest 雙手 clutch / IK；工作資料集中在本專案。
- 已完成：`quest_teleop/ethernet_rebel.py` 提供兩臂獨立 CRI + Position Interface、查詢埠號、弧度／角度轉換、只傳六個手臂軸、啟動檢查、ACK／錯誤處理、命令看門狗與雙臂故障停流。
- 已完成：`quest_dual_arm_teleop.py --backend real` 使用 Ethernet；`--backend real-ros` 保留舊單臂 topic 版本；模擬仍使用 `isaac`。實體版從實測姿態開始，禁止自動 home；tracking loss 後須放開握把再握持。
- 已完成：GUI 的雙臂選擇、左右 IP 與首次 dry-run 預設；`config/real_arms.json` 為 CLI 端點來源，GUI 明確輸入的 IP 另存 `config/launcher.json`。舊 launcher 設定首次讀取時複製至專案，舊預設資料夾改為本專案 `datasets/`；不搬移舊資料或使用者自選路徑。
- 已完成：README 與 HTML 手冊更新為雙臂 Ethernet 操作方式；新資料集預設專案內 `datasets/`，既有錄製功能仍僅支援模擬。
- 原有未追蹤的 `dual_rebel_task/`、`j6_swing_test.py` 屬使用者既有內容，未變更。
- 唯讀連線觀測：本機來源 `192.168.3.54`，兩臂 TCP 3920 可達；皆回報 `V980-15-000-2`、實體 `REBEL-6DOF-03`、Position Interface `3940 false false`。
- 當時左臂 `ERROR NoError` / `KINSTATE 0` / `ESTOP 3`；右臂 `ERROR _MNE`、六軸錯誤碼 4、`KINSTATE 99` / `ESTOP 3`，馬達未啟用。此為時間點觀測，啟動時須重新檢查。
- 驗證：`./quest_teleop/run_quest_real_arm.sh --check-only` 已對兩台實體控制器成功執行；時間與完整狀態在 `logs/real_connection_check.json`（2026-10-15 21:53:44 +08:00）。只送查詢、零 jog 心跳及 QUIT。
- 驗證：23 項測試通過，記錄在 `logs/ethernet_tests.log`。測試僅連接 `127.0.0.1` 的假控制器，包含完整假 Quest → IK → TCP 控制、兩臂獨立回授、放開握把／tracking loss／重新握持、速度限制、拒絕命令、斷線、feedback stale、watchdog、motor-off/passive/program-running 啟動拒絕；ROS 整合驗證實體後端不建立手臂命令 topic。
- 完整測試重跑（bash，專案根目錄）：`source ros_env.sh && source /opt/ros/humble/setup.bash && source igus_ws/install/setup.bash && .venv_quest/bin/python -m unittest discover -s tests -v`。系統 Python 可跑 TCP 測試；缺少 ROS 時整合測試會略過。代理的 sandbox 預設封鎖 socket，執行 localhost 測試也需要允許 socket 的工具權限；此非程式測試失敗。
- 驗證：隱藏 Tk 視窗元件測試確認雙／單臂選擇、IP 欄位、dry-run 預設與重複 IP 拒絕；Python 編譯、bash 語法及 `git diff --check` 通過。未宣稱 GUI 視覺排版已驗收。
- 實體限制：尚未連接真實 Quest 做運動驗證、未啟用 Position Interface 或馬達；實機 TCP 串流與方向確認待現場操作者執行。右臂仍需由操作人員處理馬達未啟用狀態。左臂回報工具設定 `knivetool1_clear.xml`，不可當成模擬 `eef` 工具已校正；預設控制點為 flange。右臂無工具時，URDF FK 與唯讀 Cartesian 觀測相差小於 3 mm，只是幾何一致性檢查，不能代替完整校正。
- 尚未實作：實體夾爪、實體相機／錄製、完整連桿與支架／桌面碰撞檢查。雙臂 guard 沿用模擬幾何。即時命令 / 回授的超時與速度限制是軟體保護，不能代替硬體急停。
- 後續起點：先跑 `--check-only` → `--dry-run`；現場人員準備馬達及彎曲工作姿態後，才以 `./quest_teleop/run_quest_real_arm.sh --arms left right` 啟動實機控制。不要自行為了驗證開發成果而啟用馬達或移動。

## 原廠協定來源

- CRI V17：<https://wiki.cpr-robots.com/images/8/81/CPR_RobotInterfaceCRI_V17.pdf>
- Position Interface V1：<https://wiki.cpr-robots.com/images/7/70/CPR_PositionInterface_V1.pdf>
- 本地舊驅動參考：`igus_ws/src/iRC_ROS/irc_ros_hardware/src/irc_ros_cri.cpp`；勿直接更改這份忽略追蹤的外部依賴來實作本專案功能。

## 審查與修改（2026-10-16，Claude）

- 依原廠 CRI V17 / Position Interface V1 文件逐項核對 `ethernet_rebel.py`：訊框、ALIVEJOG（0.2 s、全 0）、STATUS（POSJOINTCURRENT / ESTOP / ERROR + 關節碼 / KINSTATE / OPMODE）、RUNSTATE MAIN 第 6 欄、CMD Active、CMDACK / CMDERROR、GetPositionInterface → SetPositionInterface → UsePositionInterface 順序、`MSGSTART Pos J a1..a6 MSGEND`（度數、省略 E/C）皆符合。角度換算與 `Igus_rebel` ROS driver 相同（度 → 弧度，無偏移、無正負號翻轉）。
- 新增：`EthernetArms.park()` 與 `quest_dual_arm_teleop.park_on_exit()`，正常結束前回到 `--park-deg`（預設全 0），每拍不超過速度上限 90%，套用原有就緒／追隨誤差／看門狗檢查，兩臂距離 < `--min-arm-dist` 時提前停止；回位期間 SIGINT/SIGTERM 改為立即停止。GUI 勾選框「Return both arms to all-joints 0 deg before exit」（預設開），Stop all 等待回位完成，再按一次即停。
- 修正：遙控端每拍關節步長改為傳輸層上限的 90%，避免握持瞬間編碼器微小漂移觸發「per-tick joint speed limit」使兩臂一起斷線。
- 修正測試競態：兩臂看門狗同時到期時，`pair.fault` 可能在兩條連線都關閉後才寫入；測試改為等待 fault 寫入。
- 驗證（皆為假控制器，未連實體手臂）：28 項 unittest 通過（新增 park 單元／整合測試）；端對端以 127.0.0.1 / 127.0.0.2 兩台跟隨目標的假控制器執行實際程式：單次 SIGINT → 兩臂 10 s 回到 0° 並釋放；兩次 SIGINT → 中途停止；GUI Stop all → 回位完成後關閉。
- 注意：全 0° 是直立的奇異姿態；從工作姿態回 0° 為關節空間直線，未檢查與支架、桌面、相機的碰撞，現場需確認路徑淨空。（`real-ros` 現在也會啟動回 home、退出回 0°，見文末。）

## 第二輪審查（2026-10-16，Sonnet 唯讀審查 → Claude 修正）

未發現會讓手臂意外移動或無法停止的問題；以下皆為「安全方向」的失敗（跳過回位、誤停兩臂、清理指令沒送到），已修正並各有回歸測試（修正前的程式碼上會失敗）：
- Ctrl+C / Stop all 若剛好落在送命令途中，傳輸層會整個關閉 → 跳過回位。改為 SIGINT/SIGTERM 只設停止旗標，迴圈在 tick 邊界結束；`EthernetArms.send()` 遇 KeyboardInterrupt 不再關閉連線。啟動階段 SIGTERM 改為直接中止（rclpy 原本只把 ok() 設 False）。
- 握持第一拍：快照與即時讀值不同步，只有 0.067° 餘裕。`target()` 在串流開始的第一拍額外容許 0.5°；實機加入握把遲滯（≥ clutch+0.1 接合、< clutch-0.1 放開），且啟動後須先放開一次才能第一次接合。
- `abort()` 直接 close 帶未讀資料的 socket 會變 RST，QUIT 常送不到（40 次中只到 13–17 次）、`SetPositionInterface false` 偶爾遺失（持久化設定）。改為半關閉後讀到 EOF（≤ 0.3 s）再關。
- 回位途中的 OSError 等例外會跳過 `xr.close()`：`park_on_exit` 改捕捉所有 Exception，收尾用巢狀 finally。`--park-deg` 超出關節限位時拒絕移動。
- CRI 以 strict ASCII 解碼，非 ASCII 程式名稱會讓兩臂停機：改 latin-1；RUNSTATE 執行狀態改從欄位尾端取（程式名稱可含空白）。
- GUI：SIGTERM 3 s 後升級 SIGKILL；回位中關閉視窗會先詢問。
- 驗證：34 項測試通過；實際程式 + 兩台跟隨假控制器，SIGINT 與 SIGTERM 各一次 → 兩臂 10 s 回到 0°、釋放 Position Interface、送出 QUIT。

尚未處理（需實機確認）：
- `UsePositionInterface true` 後立即送目標，控制器的平均間隔尚未建立；可能需要先以 tick 頻率重送數次目前姿態暖機。
- 每拍上限以「拍」計而非實際時間；IK 耗時抖動可能讓瞬間速度略高於名目值（0.9 係數僅部分抵銷）。
- 追隨誤差用 Position Interface 的 `Pos J` 比較；若該值是插值設定點而非編碼器，檢查會失效，可再與 STATUS `POSJOINTCURRENT` 交叉比對。
- 全 0° 為奇異姿態，控制器若在接近時回報 KINSTATE 21–24，回位會在差幾度處中止（"failed, arms stopped"）。
- 每次執行會切換持久化的 `SetPositionInterface`；若可接受常開，可考慮不關。
- 單臂 ACK 慢於約 0.2 s 會觸發另一臂看門狗 → 兩臂安全停止，但不回位。

## 預設姿態（2026-10-16，使用者決定）

- home 姿態改為 (0, 0, 65, 0, 50, 0)°：`quest_arm_teleop.HOME_DEG`（模擬起始姿態、`--real-home`）、Ethernet 退出回位 `--park-deg` 預設值、MoveIt SRDF 的 `home` named state（另保留 `zero` = 全 0°）。SRDF 修改存於 `igus_ws/patches/Igus_rebel-home_pose.patch`，`setup/install_ros_ws.sh` 會套用所有 `Igus_rebel-*.patch`。
- 左臂（192.168.3.11）以舊 ROS driver + RViz 實機操控已可運作（2026-10-16）：driver 啟動時那一輪 Enable 未生效，需再呼叫 `/igus_rebel/reset_and_enable` 才變 `NoError, KinOK`。右臂由其他人使用，勿連線。

## RViz 實機事前測試與馬達啟用（2026-10-16，Claude）

- `ros2 launch real_robot/rviz_pretest.launch.py arm:=left|right [enable_motors:=false] [hardware_protocol:=mock_hardware]`（使用者要求用 ros2 launch，不用 sh）：include `igus_rebel/rebel.launch.py`（IP 取自 `config/real_arms.json`）+ 6 s 後 `ros2 service call /igus_rebel/reset_and_enable` + `move_group.launch.py`（RViz），全部以 SetEnvironmentVariable 設為 ROS_DOMAIN_ID 91、localhost only。mock 已驗證（driver 環境變數正確、/joint_states 可讀、Ctrl+C 無殘留）；啟用步驟只能實機驗證。不再檢查 Quest 遙控是否同時執行，操作者須自行先關閉。
- 使用者以 Quest 乙太網路版啟動時遇到 `_MNE`（舊 ROS driver 關閉後馬達停用）。新增 `EthernetArm.enable_motors()` / `--enable-motors`：僅在 ESTOP=3、CRI active、無程式執行、錯誤只有 MNE（關節碼 0/4、KINSTATE 0/99）時送 Reset → Enable，並等待 STATUS 變為無問題（≤ 6 s），否則拒絕。Reset 也會重置機器人的數位輸出。新增 3 項測試，共 37 項通過。
- 手冊 Real robot 頁新增「1 · RViz pre-test」分頁（按鈕燈色、一條指令流程、不動時的排查）。

## 兩臂相對碰撞防護（2026-10-16，Claude）

- 使用者提供手眼校正 `config/calibration/dualcal_20261012_left_hovercorr.calib`、`dualcal_20261009_right_single.calib`（camera_link 在各臂 base_link，皆標示未驗證）。`config/arm_calibration.json` 整理兩者；由此推得右臂底座在左臂座標系 (−0.007, −0.448, 0.005) m、約 3° yaw / 1° roll（文件記載實體臂距 0.448 m 吻合；模擬 USD 為 0.460 m、無偏轉）。
- 新 `quest_teleop/arm_collision.py`：每臂以膠囊表示（J1→J2→…→J6→法蘭，法蘭→XEG-32 夾爪中心、法蘭→刀尖；半徑 `--arm-radius 0.06`、`--tool-radius 0.04`），兩臂表面最小間距 < `--min-arm-dist`（預設改為 0.05 m 表面間距；舊版為 12 cm 點距離）且下一步會更近時拒絕該步並要求重新握持。實體後端用校正底座（world = camera_link），模擬用 USD 底座。回位（park）也使用同一 guard。
- Ethernet 只選一臂時，另一臂以唯讀 CRI 連線（`watch_arm()`：只讀 STATUS，若成為 active 立即 `SetActive false`，從不設定 Position Interface、不送 Reset/Enable）提供姿態給 guard；其狀態逾時則被驅動的手臂保持不動。`--no-watch-other` 關閉（無兩臂保護）。
- 驗證：segment 距離函式 300 組隨機暴力比對；新增 3 項整合／單元測試（左臂被假 Quest 推向右臂：有旁觀時停在 4.7 cm、旁觀臂只收到查詢與 SetActive false；不旁觀時越過 5 cm；校正臂距 0.448 m、home 間距 > 0.2 m、J1 −90° 伸展時重疊），共 40 項通過。未在實機驗證。未建模：支架、桌面、相機；膠囊半徑為估計值。
- 使用者決定（2026-10-16）：只開一臂時，另一臂預設**假設維持全軸 0°**（`--other-arm-deg`，預設全 0），不連線其控制器（因為常常不會兩臂都開）。唯讀旁觀改為選用 `--watch-other`。測試：左臂推向假設直立的右臂時停在 4.9 cm，右臂控制器未收到任何連線；`--watch-other` 時停在 4.7 cm。40 項通過。

## 實機第一次握持即斷線 + 實機 RViz（2026-10-16，Claude）

- 實機錯誤：`right: unexpected Position Interface message: ['OK']`。RobotControl V15 的 Position Interface 對每個目標回 `MSGSTART OK MSGEND`（V1 文件未記載），舊程式視為未知訊息而讓兩臂故障停止，所以一握持就斷線、手臂沒動。修正：`OK` 略過；`ERROR …` 視為拒絕並停止；其他未知訊息忽略但記錄在 `unknown_position_messages`。假控制器現在預設對每個目標回 OK（更貼近實機），新增 2 項測試。
- 注意：該次使用者同時選了左右兩臂，右臂也被 Reset + Enable（先前右臂由他人使用）。
- 實機 RViz：teleop 發布 `/quest_teleop/joint_states`（兩臂量測關節；未驅動臂為假設／旁觀姿態）、`/quest_teleop/{side}/hand`（PoseStamped，frame `quest` = 操作者座標系）、`/quest_teleop/state`（JSON 握持狀態）。`teleop_viz.py --source real`：world = 左臂 base_link，右臂底座由手眼校正放置，控制器 TF `{side}/controller`，綠／白／紅球表示握持／放開／追蹤遺失。GUI Real 模式勾選「RViz: live arms + Quest controllers」（預設開）會在手臂連上後開啟，Stop all 時在回位後關閉。以兩台假控制器 + 實際 teleop + 無視窗 viz 驗證資料流（關節、TF、狀態）與 GUI 式關閉；42 項測試通過。未在實機與實際頭盔驗證畫面。

## KINSTATE 51、串流暖機、啟動回 home（2026-10-16，Claude）

- 實機（右臂）一握持即 `KINSTATE=51`（igus 文件未定義；50 = 關節位置 NaN）。推測原因：開始串流時先送種子姿態、幾毫秒後即送第一個目標，控制器以「平均間隔」內插，間隔約 ms 導致速度檢查觸發運動錯誤（Position Interface 文件：間隔短而目標遠 → kinematic error）。修正：`begin_stream()`（種子 + UsePositionInterface true）後，所有串流中的手臂一起以 tick 頻率重送目前姿態 4 次（`warmup_ticks`）再跟隨目標；第一個目標在送出任何東西前先驗證。尚未實機驗證此推測。
- `enable_motors` 也會 Reset 鎖住的運動錯誤（KINSTATE 13/14/50/51/99，關節碼僅 0/4）；`problem()` 附上 KINSTATE 文字說明。
- 使用者決定：Ethernet 連上後先以 10°/s 移到 home (0, 0, 65, 0, 50, 0)°（`start_home()`，同 park 的串流與兩臂 guard；`--no-start-home` 略過，GUI 勾選預設開），關閉時回全軸 0°（`--park-deg` 預設改回全 0）。44 項測試通過（含啟動姿態 = home、退出 = 0 的整合測試）。
- 使用者問 sh 是否 source ROS：有（ros_env.sh、/opt/ros/humble、igus_ws），但 Ethernet 控制不經 ROS，KINSTATE 為控制器回報，與 ROS 無關。

## 實機改走 igus ROS driver（2026-10-16，Claude；使用者要求「用 ROS2 topic 送 command，像 Isaac 一樣」）

- Position Interface 直連在 RobotControl V15 上：回 `OK`、CONFIG SetPositionInterface 不回覆、串流起步 KINSTATE 51（暖機後仍發生）。另寫了 CRI ALIVEJOG 速度點動模式（`--control jog`，仿 driver：2 %/(deg/s)、10 ms、0.4 ff + 1.0 誤差），假控制器測試通過但未上實機；直連整條路徑保留為實驗選項 `--backend real`。
- 預設實機路徑改為 `--backend real-ros`：`real_robot/dual_arm_driver.launch.py`（每臂一組 ros2_control_node / RSP(frame_prefix) / spawner，namespace /left、/right，controller.yaml 改以 `/**/` 為鍵；6 s 後 `/<side>/igus_rebel/reset_and_enable`；ROS_DOMAIN_ID 42 localhost）。teleop 讀 `/<side>/joint_states`，以 30 Hz 單點短軌跡送 `/<side>/rebel_arm_trajectory_controller/joint_trajectory`；啟動回 home、退出回 0°（`move_to()`，含兩臂 guard）。`run_quest_real_arm.sh` 預設 real-ros。GUI Real 模式：先啟 driver（等 success=True × 臂數）→ teleop → RViz；Stop all：回 0° → teleop → viz → driver。
- **實機驗證（使用者授權、確認路徑安全）**：兩臂 driver 啟動並啟用 → teleop real-ros 將兩臂移到 home（左 64.2/49.2°、右 64.1/49.3°）→ Ctrl+C 回 0°（殘差 < 1°）。未接 Quest 實際遙控；GUI 流程以 mock driver 驗證。48 項測試通過。
- 遺留：driver 連線時本身會 SetActive/Reset/Enable（dry run 也會）；JTC goal tolerance 使回位殘差 ~0.7–1°。

## 執行中切換旋轉座標系（2026-10-16，Claude）

- 使用者要求：連線後的執行畫面多一個按鈕，選擇 tool frame 或 base frame。沿用既有 `--rot-frame tool|base`（只影響旋轉；平移一律沿手臂底座軸）。
- GUI 執行頁新增「Rotation: TOOL/BASE frame」按鈕（模擬與實機皆有）：經 teleop 的 stdin 送 `rot_frame base|tool`（GUI 啟動 teleop 時開 stdin pipe，同 recorder 做法）；按鈕標籤依 teleop 的確認日誌 `rotation frame: …` 更新，確認前停用；選擇寫入 `config/launcher.json` 供下次啟動。終端機啟動時可用 `ros2 topic pub --once /quest_teleop/set_rot_frame std_msgs/msg/String "data: base"`。
- 安全：握持中的手臂保留原座標系，放開再握才套用（兩種映射對同一手勢給出不同目標，中途切換會讓工具擺動）。
- 驗證：新增整合測試（握持中切換不生效、重握後生效、錯誤指令忽略）與隱藏 Tk 測試（按鈕送指令、依確認更新、存設定）；50 項測試通過。未在實機或實際頭盔上試手感。

## 遙控卡頓 / 封包堆積排查（2026-10-19，Claude）

- 使用者回報：模擬遙控前段順、後段卡頓。檢查整條路徑 Quest → teleop → `/dual_arm/isaac_joint_commands` → Isaac → `/dual_arm/isaac_joint_states` → teleop。
- 確認的堆積點 1（Isaac 命令端）：`ROS2SubscribeJointState` 每個 tick 只取佇列中一筆，預設 queueSize 10。模擬 FPS 低於 teleop 30 Hz 時命令排隊，手臂逐筆執行舊命令、落後最多 10 筆。實測（headless、連發 9 筆舊目標 + 1 筆新目標）：queueSize 10 時手臂先往舊目標偏 3.70°，改 1 後 0.00°。`run_dual_arm_sim.py` / `dual_arm_rig_ros2_bridge.py` 改為 queueSize 1。
- 確認的堆積點 2（teleop 狀態端）：主迴圈每拍 `rclpy.spin_once` 只處理一個 callback，Isaac 每個渲染幀發布狀態，depth 10 佇列塞滿後每次取到的都是最舊的一筆，`q_meas`（握持快照、碰撞 guard、real-ros 落後檢查）落後約 10 筆。改為 depth 1（sim 與 real-ros 皆是）。回歸測試：舊設定落後 11 筆，新設定為最新。
- 使用者 2026-10-19 18:38 存的 `dual_arm_rig_v2.usda`（桌子升高 0.2 m：桌腳 scale z 0.7→0.9、桌面 z +0.2）連帶存進了腳本建立的 `/World/DualArmJointBridge` 圖（舊 queueSize），會讓啟動報 "A graph already exists"。使用者決定採用此 USD：已移除檔內的圖再 commit，headless 載入驗證正常。`run_dual_arm_sim.py` 另在載入後移除任何存進 USD 的 bridge 圖再重建（只改記憶體），以免日後再次存檔又出錯。
- 新增 `[timing]` 診斷（每 `--diag-period` 5 s 一行；異常時為 WARN，GUI 也顯示）：迴圈 Hz / overrun、Quest 讀取 / IK / 發布耗時 p95、joint state 頻率與年齡、模擬速度（sim 時間 ÷ 牆鐘）。判讀：loop < 27 Hz 或 ik 高 → teleop 本身忙；`sim < 90 % real time` 或 joint states Hz 下降 → Isaac 被 GPU/CPU 拖慢；age 高 → 狀態回授延遲。
- 已排除：isaacteleop `ControllerTracker` 取當下快照，不是 FIFO；GUI 的 `_pump` 執行緒持續讀 teleop stdout，管線不會塞住（`Proc.lines` 會整場累積，只佔記憶體）。
- 觀察：headless 模擬沒有限速（約 260 Hz、220 % 真實時間）；GUI 模式受渲染限制，實際數字未量。CPU governor 為 `powersave`（Isaac 啟動時也警告），可能讓模擬或 IK 後段變慢，尚未證實。
- 驗證：53 項測試通過（新增 3 項）。未用實際頭盔重跑長時間遙控，「卡頓已消除」尚待使用者以新版跑一次並看 `[timing]` 行確認。

## 紙箱任務場景（2026-10-19，Claude）

- 使用者需求：wrist 相機連桿會晃；箱子要四片蓋（原本只有前後兩片）；箱子在桌面範圍內隨機生成；任務 = 看到箱子 → 移到中間 → 打開四片；箱子要輕但不能一碰就飛。
- 做法：`dual_arm_sim/setup_box_task_scene.py`（`~/isaac-sim-6.1/python.sh` 執行，原地改 `dual_arm_rig_v2.usda`，可重跑、結果不變；pxr 讀寫此 USD 不會改格式）；隨機擺放在 `run_dual_arm_sim.py` 執行時處理。
- wrist 相機晃動根因：`xeg32_camera_mount_link` 質量 1e-9 kg、`camera_tilt_joint` 無剛性無阻尼（自由擺）。改為 0.05 kg、慣量 3e-5、角度 drive stiffness 20 N·m/deg、damping 1 N·m·s/deg、目標 0°（`--cam-tilt-deg` 可改），仍為 revolute（DOF 名單不變）。實測 joint1 ±45° 擺動時相機傾角峰對峰：改前 19.35°，改後 0.000°。
- 箱子：Body 由 kinematic 改為 dynamic、0.4 kg；PhysX：maxDepenetrationVelocity 0.5 m/s、maxLinearVelocity 2 m/s、線阻尼 0.1、角阻尼 0.5、solver 16/4、CCD。物理材質（原本沒有）：紙箱靜/動摩擦 0.6/0.5、桌面 0.5/0.4，回彈 0。
- 四片蓋：長蓋 LidBack/LidFront（0.157×0.45 m、0.035 kg）抬高 6 mm 疊在新增的短蓋 LidLeft/LidRight（0.30×0.12 m、0.02 kg）上，間隙 1 mm，所以要先開長蓋。鉸鏈：關閉 = 限位 0°（不會掉進箱內），全開 175°（往外翻平；PhysX revolute 限位超過 ±180° 會亂動，265° 時實測短蓋被推開），無彈簧，damping 0.0002 N·m·s/deg（原本 5，相當於 286 N·m·s/rad，蓋子幾乎推不動）。未過垂直放手會倒回關閉，推過垂直會倒向全開。
- 隨機擺放：每次啟動在桌面內取中心與 yaw（USD 方向 ±30°），四角距桌緣 ≥ 2 cm、中心距桌面中心 ≥ 5 cm；`--box-seed` / `--no-random-box` / `--box-yaw-deg` / `--box-margin` / `--box-min-offset`。執行中 `ros2 topic pub --once /dual_arm/reset_scene std_msgs/msg/Empty "{}"` 重新擺放（全部剛體一起移動、速度歸零、蓋子關閉）；`/dual_arm/box_pose` 發布 Body 位姿（frame world）。桌面中心 (0, −0.663)。
- 驗證（headless，皆為模擬）：`check_box_task_scene.py` 全部通過 —— 靜置 5 s 漂移 0.00 mm、四蓋 < 0.05°；四蓋 60° 放手回 0°、110° 放手到 175°；側推 0.3 m/s 滑 0.5 cm 不傾倒；壓入桌面 1 cm 的彈出速度 0.25 m/s（上限 0.5）；相機峰對峰 0.000°。20 次 reset 皆直立、在桌面上、靜止。home 姿態手臂最低點 1.196 m 高於箱頂 1.191 m。53 項 unittest 通過。
- 未驗證：實際用手臂/夾爪推箱子與開蓋的手感（需 Quest 實操）；遠側桌面接近 ReBeL 工作範圍邊緣。錄製尚未在每個 episode 自動 reset 箱子。既有 `ch_v4_*_hold` 固定關節每次啟動警告 "disjointed body transforms"（約 0.2 m z 差），與本次無關、未處理。

## 模擬卡頓判讀 + 模擬用 RViz（2026-10-19，Claude）

- 使用者實測的 `[timing]`（logs/teleop_20261019_193103.log）：teleop 迴圈 29.9 Hz、tick p50 約 1–2 ms、Quest 讀取 1.5 ms，皆正常；但 Isaac joint states 只有 **15 Hz**（GUI 渲染約 15 FPS），模擬速度從 65 % 降到 **35–40 % 真實時間**。卡頓主因在 Isaac 端（渲染 / CPU），不在 teleop。第二段下降約在 19:32:08，與 sim log 中使用者操作選單（開 Viewport）同時。GPU 當時未降頻；CPU governor 為 powersave；AnyDesk 在執行（遠端桌面擷取畫面也吃資源）。
- RViz 模擬版：GUI 勾選「RViz (sim)」（預設開）或 `quest_teleop/run_teleop_viz.sh --source isaac`。實心手臂 = Isaac 實際姿態，半透明 = teleop 送出的命令（`/quest_teleop/action` → `/cmd_{side}/joint_states` + 兩個 robot_state_publisher），球 = Quest 控制器（綠 = 握持）。畫面上有延遲面板並每 5 s 寫入 viz log：Quest 端（手部姿態 Hz、逐位元相同的凍結幀 %、最長間隔）與手臂端（命令 Hz、手臂狀態 Hz、模擬速度、手臂落後命令的最大角度），並給出判斷。
- 驗證：headless 模擬 + 假 teleop 發布：ghost 手臂 TF、控制器 TF、凍結偵測（凍結 2 s 時 83 %）、無狀態時提示；53 項測試通過。未在實際頭盔下看過 RViz 畫面。

## 卡頓再查：不是 GPU，是 CPU 主執行緒（2026-10-19，Claude）

- 使用者質疑顯存用量低。實測模擬執行中：GPU 35 %、P5、960 MHz、Isaac 顯存 1.5 GB → GPU 不是瓶頸；Isaac 主執行緒 CPU 87–100 % → 每秒約 98 幀，每幀只推進 1 個 physics step（1/120 s，由 `/clock` 量得 8.3 ms/幀），所以要 120 FPS 才是真實時間，量到 82 %；teleop / Viewport / RViz 同時跑時降到 35–65 %。
- 更正：先前 `[timing]` 的「joint states 15 Hz」是 teleop 量測假象 —— 每拍一次 `spin_once` 只有隔拍處理到狀態。實際 Isaac 發布約 95 Hz。teleop 改為每拍 spin 3 次（假 100 Hz 發布實測 15.0 → 29.7 Hz），q_meas 也因此每拍更新。
- teleop 雙手握持時 IK 區段 p95 達 40–60 ms 並 overrun；離線同樣的 IK 兩臂只要約 4 ms（在 Isaac 執行中量測 p95 4–10 ms）。live 尖峰原因未確定。teleop 的 2 條 OpenBLAS 執行緒在空轉（各約 62 s CPU，多為 sys）；設 `OPENBLAS_NUM_THREADS=1` 時 CPU 用量從 7.2 s 降到 1.2 s，但 IK p95 反而變差（9–10 ms 對 4–6 ms，推測與 powersave 下核心降頻有關），所以未採用。

## CPU 分核 + 計時 log（2026-10-19，Claude）

- 使用者已把 governor 改為 performance。另發現 `intel_pstate/no_turbo = 1`：i9-14900K 的 Turbo 被關，P-core（0–15）上限 3.2 GHz、E-core（16–31）2.4 GHz。Isaac 受單一主執行緒限制，頻率影響直接。是否開 Turbo 由使用者 / 管理者決定（14900K 有已知的高頻穩定性問題，可能是刻意關的；開之前先確認 BIOS / microcode）。
- GUI 啟動的程式依角色綁 CPU（`taskset`，子程序繼承；`ROBOT_REMOTE_NO_PIN=1` 關閉）：sim = 最快的核心扣掉最後兩個實體核（本機 0–11），teleop（含 CloudXR runtime）= 那兩個實體核（12–15），RViz / robot_state_publisher / 錄製 / driver = 慢核（16–31）。每個 log 第一行記錄綁定的 CPU。終端機手動執行時可用 `taskset -c 0-11 ./dual_arm_sim/run_dual_arm_sim.sh`、`taskset -c 12-15 ./quest_teleop/run_quest_dual_arm.sh ...`、`taskset -c 16-31 ./quest_teleop/run_teleop_viz.sh`。
- Isaac 新增 `[dual_arm] [timing]`（每 5 s）：frames/s、frame p50/p95/max、模擬速度、主執行緒所在 CPU。headless 綁 0–11 實測 131 frames/s、frame p50 6.8 ms。
- teleop `[timing]` 的 ik 區段拆出 `solve`（IK 解算本身，兩臂合計，含 max）與 `guard`（兩臂碰撞檢查），並記錄主執行緒所在 CPU。用來判斷先前 40–60 ms 的尖峰是 IK、guard 還是其他。
- 驗證：53 項測試通過；pinning 以子程序讀 `Cpus_allowed_list` 確認。尚未在實機操作下看到新的 log。
- 修正（2026-10-19）：teleop 啟動後握持即崩潰 `ValueError: Logger severity cannot be changed between calls.` —— `[timing]` 用同一行 `(log.warning if bad else log.info)(line)`，rclpy 不允許同一呼叫位置改變嚴重度，第一次由 WARN 轉 INFO 就丟例外。改為 `log_timing()` 兩個呼叫位置，新增回歸測試（舊寫法重現該錯誤），54 項測試通過。

## 箱子黏在手臂上：排查紀錄（2026-10-19，Claude；未修改場景）

- 使用者回報：推箱子後手臂離開，箱子跟著手臂走。Codex 調查已依使用者要求取消，未改動專案。
- headless 重現（PhysX contact report，左臂夾爪朝下、手指全開 = teleop 未按 trigger）：**側推後收回不會黏**（推 7.7 cm、收回只滑 0.7 cm、不跟隨，所有變體皆同）。
- 會拖動箱子的情境：(1) 夾爪被命令壓進已關閉的蓋子 2.5–3 cm（箱在桌上無處可退，15000 N·m/deg 的位置驅動等於無限大的力 → 穿過 5 mm 蓋子、被蓋子從下方勾住）；(2) **長蓋開到 175° 時是一片在箱頂高度、朝手臂伸出 16 cm 的硬棚板**，從棚板下推牆再離開時，夾爪／手肘勾住棚板（contact：LidBack↔夾爪、LidFront↔motor3），箱子被拖 3–4 cm。
- 無效或更糟、不要採用的變體：關相機支架碰撞（改由 xeg32_base_link 勾住）、maxDepenetrationVelocity 10、contactOffset 2 cm、手臂驅動剛性降為 1500、鉸鏈軟限位（PhysxLimitAPI k 0.05／1：蓋子被翻到 45–125°，一次把箱子抬高 7 cm）、全開限位 110°（立起的蓋子被手臂掃到）。
- 測試陷阱：IK 軌跡第一步若直接要求夾爪朝下，關節會在一幀內轉 86°，手臂甩過箱子，看起來像「沒接觸就動」。改為姿態 slerp、每幀關節 < 1.6°。手指目標未設定時停在 0（半閉），會夾住 6 mm 牆。
- 下一步：請使用者重現時錄 `ros2 bag record /quest_teleop/action /dual_arm/isaac_joint_states /dual_arm/box_pose /quest_teleop/state -o logs/sticky_box`，用實際命令在 headless 重播並看接觸對；並確認當時蓋子是否已打開。
- **使用者決定（2026-10-19）：箱子固定不動，只有四片蓋子可開合**，以避開箱子被手臂拖動等問題。`setup_box_task_scene.py` 把 Body 設為 kinematic（已套用到 USD）；`run_dual_arm_sim.py` 預設箱子在 USD 位置 (0, −0.55)，`--random-box` 才在啟動時隨機擺放；`/dual_arm/reset_scene` 改為把四片蓋子關回去。驗證：`check_box_task_scene.py` 全過（箱身 0.00 mm）；先前會拖動箱子的四種手臂軌跡（側推、壓牆緣、壓蓋子、從開蓋下方推）箱身位移全為 0；reset 實測關閉蓋子。蓋子仍可能被手臂撐住或勾起（屬開合動作）。

## 遙控器重置場景（2026-10-19，Claude；使用者要求，錄製時不用中斷）

- 按鍵：A/X = 開始／儲存 episode，B = 丟棄，**Y（左手）= 重置場景**（原本 B/Y 都是丟棄）。GUI 執行頁另有「⟲ Reset scene (= Y)」按鈕（經 teleop stdin `reset_scene`）。
- 重置（僅模擬）：送出 `reset` 事件（錄製中的 episode 由 record_dataset.py 丟棄）→ 兩臂放開、夾爪打開 → 以 0.4 rad/s 回到 HOME_DEG → 發布 `/dual_arm/reset_scene` 讓模擬關上四片蓋子（手臂離開後才關）→ 重新握持即可繼續。實機後端按 Y 只記錄警告，**不會因按鍵移動實體手臂**。
- 驗證：新增整合測試（假 Isaac 跟隨命令、假 Quest 握持中按 Y）：握持被放開、`reset` 事件與 `/dual_arm/reset_scene` 各一次、手臂回到 home ±1°；55 項測試通過。未用實際頭盔試按。

## OpenArm 模擬改造（2026-10-21，Claude；分支 `feat/openarm-sim`）

- 使用者 fork（jonathan0713/robot_remote）後要改成遙控 ITRI_project 的 OpenArm 雙臂（7 軸 + 夾爪），先只做 Isaac Sim 5.1 模擬，可以大幅修改。計畫在 `docs/openarm_sim_port_plan.md`。
- 方向：robot_remote 只負責 Quest → 關節命令，並加上 robot profile（ReBeL 和 OpenArm）；場景、錄製、匯入沿用 ITRI chain flow 的 `TOTE_CHAIN_ASSIST`（`/right_joint_command` + `capture.fifo`），資料格式和 held-tote v5 一致。
- 目前狀態（2026-10-21）：P0 環境已裝好，P1 完成；尚未啟動 Isaac，也沒有連過實際的 Quest。第 6 節的問題待使用者回覆。

### P0 / P1 進度（2026-10-21，Claude）

- P0：經使用者同意，`setup/install_quest_env.sh` 已建立 `.venv_quest`（isaacteleop 1.3.131、numpy 1.26.4、scipy 1.15.3，227 MB）。CloudXR EULA 尚未接受：第一次在終端機執行 `quest_probe.py` 時會出現連結並詢問 `[y/N]`，由使用者本人閱讀後回答。
- `quest_probe.py` 已改寫：舊版依賴本機沒有的 `~/Robotic_infra/lerobot_isaac`。新版直接用 `quest_input.QuestController` 讀兩支控制器，不需要 ROS，也不需要機器人，每 5 s 印一次 pose 更新頻率和按鍵次數。
- 拆出不依賴 ROS 的模組：
  - `quest_teleop/kinematics.py`：`SerialChain` 改成 n 軸；不在 `joint_names` 裡的 revolute 關節視為 fixed；姿態任務只在每次呼叫的第一個 iteration 加入。
  - `quest_teleop/quest_input.py`：`QuestController`，可指定 app_name。
  - `quest_teleop/teleop_core.py`：`ArmTeleop`，內容是 run_session 的單臂數學；`tool_from_controller` 多了 `tool_axis` 參數。
  - `quest_teleop/robot_profile.py` + `config/robots/openarm_dual_gripper.json`。
  - `quest_arm_teleop.py` 與 `quest_dual_arm_teleop.py` 改為從這些模組 import，ReBeL 行為不變（6 軸 FK/Jacobian 完全相同，IK 差 2e-14）。`arm_collision.py` 改成延遲 import ReBeL 的 TCP 預設，並新增通用的 `link_capsules()`。
- 新的 OpenArm teleop：`quest_teleop/quest_openarm_teleop.py`，用 `run_quest_openarm.sh` 啟動（套用 chain 的 ROS 環境：domain 42、fastrtps profile、不設 LOCALHOST_ONLY）。
  - 每個 tick 只發布被驅動手臂的 `/<side>_joint_command`（7 軸 + finger_joint1）。
  - 起始命令取 `/joint_command` 上最後的目標，避免下垂。
  - 夾爪採軟接手：trigger 到達目前命令附近之後才開始跟隨。
  - 兩臂膠囊 guard：未驅動的手臂用量測值。
  - 按鍵：A/X 寫 `capture.fifo`，B 寫 `teleop_discard.json` 並 touch `release`，Y 只 touch `release`。
  - 尚未實作：`measurements.json`（P2）。
- 實測：
  - OpenArm FK 與 TurboVLA `ArmKinematics` 相差 1e-15。
  - 往外的工具軸方向：左臂是 ee_tcp 的 +Z，右臂是 −Z。
  - ready → 箱內 → ready 的路徑：沒有姿態任務時，回到 ready 後關節漂移達 25°；姿態 gain 0.05 時漂移 ≤ 0.2°，目標誤差 0.1 mm / 0.00°，IK 每 tick 0.9 ms。
  - held-tote 起始狀態的膠囊間距 0.087 m，與 ITRI sphere 模型一致。
  - 從 j1 原點量的最大 reach 0.69 m，profile 的 max_reach 設 0.62 m。
- 測試：
  - 新增 `tests/test_openarm_kinematics.py`（10 項，純 numpy）和 `tests/test_openarm_teleop_ros.py`（1 項：假 Quest + 假 Isaac + FIFO，跑在隨機 ROS domain）。
  - 全套 66 項中 13 項 error，全部是 `igus_rebel_description` 找不到（本機沒有 build igus_ws）。乾淨的 HEAD 匯出版本有相同的 13 項 error，所以不是這次修改造成的。
- 下一步：使用者跑 `quest_probe.py`（P0 驗收）→ P2（ITRI 端的 `run_held_tote_teleop.py` glue 和 `measurements.json`；啟動 Isaac 前需使用者同意）。

### Quest 在遠端：網路 bridge（2026-10-21，Claude）

- 狀況：
  - 主機的 Sunshine（Moonlight 串流主機）佔用 UDP 47998。這是 CloudXR StreamSDK 寫死的 port：`NV_CXR_MEDIA_PORT` 和 `NV_CXR_SERVER_PORT` 都改不到它，已實測。
  - 使用者正透過 Sunshine 遠端操作，不能停掉它。
  - Quest 在使用者那一端，旁邊有一台跑 Patrick 版本（igus、Isaac 6.1）的 4060 筆電，CloudXR 在那台已經能用。
- 做法：新增 `quest_teleop/quest_bridge.py`。
  - 筆電跑 `send --to <主機 Tailscale IP>:47710`：CloudXR 加上兩支控制器，以 60 Hz UDP 送出 JSON。
  - 主機的 `quest_openarm_teleop.py --quest-source udp://0.0.0.0:47710` 和 `quest_probe.py --source udp://...` 改用 `NetworkQuest`，介面和 `QuestController` 相同。
  - 超過 0.2 s 沒收到新資料就視為 tracking lost，手臂放開；亂序的封包會被忽略；可以用 `--allow` 限制來源 IP。
- 測試：
  - `tests/test_quest_bridge.py`：7 項，localhost UDP。
  - ROS 測試多了一項「控制器資料經 UDP」的完整 session。
  - 全套 74 項，13 項 error 仍然只是缺 igus_ws。
- 尚未實機驗證：筆電端的 sender、Tailscale 的延遲，以及主機防火牆（ufw）是否放行 UDP 47710。


### P0 實測：Quest → 筆電 CloudXR → Tailscale → 主機（2026-10-21，Claude + 使用者）

- 架構：筆電（ASUS TUF，Tailscale 100.91.9.78）執行 `~/Documents/Jonathan/robot_remote_openarm`，它的 `.venv_quest` 是連到 Patrick 版本環境的 symlink；Quest 連筆電的 CloudXR；主機（100.89.140.123）用 `quest_probe.py --source udp://0.0.0.0:47710` 接收。
- 連線：兩次合計約 1.7 萬個封包，**0 遺失**；兩手 pose 更新 57–59 Hz；最後一筆封包都在 15 ms 內。squeeze 和 trigger 都能到 0–1；A、B、X、Y 四個按鍵都收到。靜止時抖動 1–2 mm。
- 頭盔不需要戴，但它是控制器的追蹤器：控制器必須在頭盔相機的視野內，而且頭盔休眠時兩手都會變成 not tracked。第一次把頭盔從頭上拿下來改放到胸口時，追蹤就一直沒恢復。
- 方向：往上 z +32 cm ✓；往前是 x +23 cm 加上 y −11 cm；往左是 y +14 cm 加上 x +7 cm。也就是說三個方向的正負號都對，但前後左右兩次測試都有約 **26° 的偏角**，頭盔換了位置也一樣。長按 Meta 鍵重新置中後偏角仍在，推測這個參考方向跟 guardian / stage 有關。補償方式：teleop 加 `--yaw-deg 26`，但每次換房間或重設 guardian 都要重新確認。
- 結論：P0 通過。
