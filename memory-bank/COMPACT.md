# robot_ws_3 — Ngữ cảnh gọn (đọc trước)

**Là gì:** AGV mini cho đồ án. Mỗi xe: ESP32 + L298N + 2 động cơ encoder + YDLIDAR X2 +
BNO055, micro-ROS qua Wi-Fi (UDP 8888) tới laptop **WSL2 Ubuntu 24.04 + ROS 2 Jazzy**.
Laptop: odometry, AMCL trên bản đồ có sẵn, Nav2 (Regulated Pure Pursuit), logic sa bàn.
Repo `~/robot_ws_3`, GitHub `NguyenHuynh-2004/Robot_kho` (đẩy bằng git Windows, chỉ khi được bảo).

**Bài toán:** sa bàn **5×5, ô 60 cm** (hàng 1–5 dưới→trên, cột 1–5 trái→phải). Nhập hàng, cột,
hướng (Đ/T/B/N); xe chỉ đi trên cạnh, xoay tại chỗ, né nút/cạnh bị chặn. **Đạt = tại đích
≤ 3 cm, ≤ 3°.** Hai xe do `traffic_manager.py` điều phối (giữ chỗ 2 nút, ưu tiên xe gần đích,
gỡ kẹt).

## Chạy
| Kịch bản | Lệnh chính |
|---|---|
| 1 Vẽ bản đồ (tay cầm) | `bringup_real.launch.py xe:=N`, SLAM, joy bridge (xem docs) |
| 2 Một xe thật | `ros2 launch my_bot_bringup bringup_real.launch.py xe:=N` → `navigation.launch.py use_sim_time:=false` → `python3 tools/auto_initial_pose.py` → `python3 grid_navigator.py --real` |
| 3 Hai xe mô phỏng | `tools/fleet_sim.sh up` (domain 87) → `ROS_DOMAIN_ID=87 python3 traffic_manager.py agv1 agv2`; RViz `tools/fleet.rviz`; Gazebo xem/đặt vật cản `tools/fleet_gazebo.py` |
| 4 Hai xe thật | xe 1 ở (3,1)→Đông, xe 2 ở (3,5)→Tây; `tools/fleet_real.sh up` → `python3 traffic_manager.py agv1 agv2`; `tools/fleet_real.sh down` |
| **Tự động (trợ lý chạy)** | `tools/agvctl up` (chờ xe bật nguồn → tự bật hết) → `agvctl status/errors/events` → `agvctl goal "..." --go` → `agvctl down`. Xem `docs/AGVCTL.md` |

Tài liệu người dùng: `docs/HUONG_DAN_CHAY.md`, `docs/SA_BAN_VA_TAI_LIEU.md` (tra sự cố,
nối dây, thuật toán, quy tắc 2 xe, nạp firmware H3).

## Kiến trúc then chốt
- Firmware `firmware/esp32_agv_base/esp32_agv_base.ino`, chọn xe bằng **`#define XE 1|2`**
  (`agvctl fw build N` biên dịch từ WSL bằng arduino-cli của IDE; `/agvN/diag` JSON 1 Hz,
  `/agvN/estop_reset`).
  Node micro-ROS `/agvN/esp32_agv_base_N`; topic **`/agvN/scan`, `/agvN/imu/data`,
  `/agvN/joint_states`** (góc bánh tích luỹ), nhận **`/agvN/cmd_vel`**. ESTOP chốt tới reset
  (kẹt bánh / quay ngược chiều PWM); mất cmd 0.5 s thì dừng.
- **Xe 1 vs xe 2:** encoder 23/34/4/35 vs 23/18/19/17(kéo lên); PWM LiDAR 18 vs 4; động cơ xe 2
  **nối chéo** (L = 33/32/14, R = 27/26/25); LiDAR xe 2 nhân khoảng cách **1.062**.
- 1 xe: `ns_relay.py` (trong bringup_real) chuyển `/agvN/*` ↔ gốc, phần còn lại như cũ.
- 2 xe: mỗi xe cây TF riêng `/agvN/tf`; `fleet_real_robot.launch.py` (rsp, `wheel_odometry.py`,
  AMCL `map_topic:/map`), Nav2 **gộp một container/xe** (`tools/fleet_nav.launch.py`, nạp file
  tham số cho container), `fleet_agent.py` (`SingleThreadedExecutor`, lọc vật cản 0.25 m quanh
  xe kia, `/fleet/status` 5 Hz, `/fleet/agvN/cmd`). `/map` và `/fleet/*` dùng chung.
- `grid_navigator.py`: Dijkstra (ít cạnh → ít rẽ) → đoạn thẳng → action `follow_path`; xoay tại
  chỗ theo **gyro**, `coast 0.36` (đo xe 1), bù ≤ 5 lần. Chặn nút/cạnh tính lại mỗi local costmap
  (**`always_send_full_costmap: True`, 5 Hz**).
- Cấu hình: `src/my_bot_bringup/config/{navigation.yaml, grid_real.yaml, real_room_map.yaml}`;
  `wheel_radius 0.03281`, cách bánh 0.200; `xy_goal_tolerance 0.02`.

## Môi trường & bẫy
- bashrc: `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (bắt buộc), `GALLIUM_DRIVER=d3d12`,
  source `~/microros_ws/install` (agent tự build). WSL mirrored networking; laptop 192.168.1.21.
- `colcon build --symlink-install`; file mới trong `scripts/` cần build lại.
- **Sửa file qua `\\wsl.localhost` mất bit chạy → `chmod +x`.** Gọi WSL từ Git Bash:
  `MSYS_NO_PATHCONV=1 wsl.exe -e bash script.sh`; `/tmp` hai bên khác nhau.
- Quá ~30–40 tiến trình ROS/domain thì DDS LOCALHOST không thấy tiến trình bật sau.
- `MultiThreadedExecutor` + topic 50 Hz → CPU 80–90%.
- `ros2` CLI dò tìm thiếu khi nhiều tiến trình → kiểm tra bằng script rclpy.
- `sed` với `C:\Users` (`\U`,`\L`) và vá bằng Python không-raw (`\n`,`\1`) làm hỏng file.
- RViz lỗi đỏ `GLSL ... indexed_8bit_image` → `sudo tools/fix_rviz_shader.sh`.
- Thử 2 xe không cần phần cứng: `fleet_fake_robot.py ... --esp32` + `AGENT=0 EMULATE=1 tools/fleet_real.sh up` (domain riêng, vd 89).

## Quy tắc làm việc với người dùng
Tiếng Việt. **Người dùng tự nạp firmware** (giữ BOOT); trợ lý biên dịch kiểm tra, chép sang
`D:\CTU\Ardunio ide\code\Xe_lidar\Xe_lidar.ino`, đọc log. Làm từng giai đoạn, chờ "xong".
**Không gửi `/cmd_vel` tới xe thật khi chưa được phép.** Không commit `wifi_config.h`.

## Trạng thái (2026-10-10)
Hai xe thật: code + tài liệu xong, đã thử với ESP32 giả. Đã có môi trường vận hành tự động
(`agvctl`, `fleet_monitor`, MCP `ros`, `/diag`) — thử đủ vòng với ESP32 giả (domain 89).
**Chờ:** người dùng `sudo apt install ros-jazzy-rosbridge-server`; nạp firmware mới (có `/diag`)
cho cả 2 xe. Xe 2 đang dùng `wheel_radius`/`coast` của xe 1. Chi tiết: `activeContext.md`,
`progress.md`.
