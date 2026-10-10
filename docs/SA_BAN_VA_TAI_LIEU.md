# Sa Bàn, Sự Cố Và Tài Liệu Kỹ Thuật

Tài liệu tham khảo đi kèm [HUONG_DAN_CHAY.md](HUONG_DAN_CHAY.md) (file đó chỉ giữ các
bước chạy). Ở đây: sa bàn 5×5 và cách căn chỉnh, bảng tra sự cố, nối dây, thuật
toán, quy tắc hai xe, và các hướng dẫn ít dùng (chuẩn bị máy mới, Gazebo, nạp
firmware, kiểm tra định vị).

---

## 🧭 Sa Bàn 5×5

Đánh số **từ 1**: **hàng 1 = dưới cùng**, **cột 1 = bên trái**. Sa bàn **cố định trên
bản đồ**, đọc từ file cấu hình — xe đứng đâu cũng không kéo sa bàn theo.

| | File | Ô | (hàng 3, cột 3) |
|---|---|---|---|
| Xe thật | `src/my_bot_bringup/config/grid_real.yaml` | 60 cm | (−0.545, +0.035) |
| Mô phỏng | `src/my_bot_bringup/config/grid_sim.yaml` | 2 m | (0, 0) |

Ảnh xem trước: [`sa_ban_5x5_preview.png`](sa_ban_5x5_preview.png) (xe thật),
[`sa_ban_5x5_preview_mo_phong.png`](sa_ban_5x5_preview_mo_phong.png).

**Điểm/cạnh sát vật cản tự bị bỏ:** điểm cách tường/đồ vật < 25 cm, cạnh < 20 cm. Xe
thật hiện bỏ **(hàng 1, cột 5)** — sát chân bàn ghế; xe tự đi vòng. Vật cản mới
(người, thùng) chặn thêm điểm/cạnh lúc đang chạy.

### Căn Sa Bàn Cho Khớp Vạch Gạch (sau khi vẽ bản đồ mới)
Cần xe đang chạy và đã định vị (Kịch bản 2 của [HUONG_DAN_CHAY.md](HUONG_DAN_CHAY.md), hoặc [H4](#h4-kiểm-tra-định-vị-trên-bản-đồ--amcl-cần-4-terminals)).

```bash
cd ~/robot_ws_3
source install/setup.bash

# 1. Góc: bản đồ SLAM có hướng tuỳ ý, đo góc tường để lưới song song vạch gạch
python3 tools/set_grid_center.py --yaw-walls --keep-center

# 2. Tâm: đặt TÂM XE đúng giao điểm vạch (hàng 3, cột 3), xem lệch bao nhiêu
python3 tools/set_grid_center.py --dry-run
#    lệch > 2 cm thì ghi lại tâm:
python3 tools/set_grid_center.py

# Hoặc đo nhiều giao điểm (lái xe tới từng giao điểm, nhập "hàng cột"):
python3 tools/calib_grid_points.py            # chỉ xem
python3 tools/calib_grid_points.py --write    # ghi vào grid_real.yaml
```
Chạy lại `grid_navigator.py --real` sau khi đổi. Vẽ lại ảnh xem trước:
`python3 tools/preview_grid.py`.

---

## 🛠️ Bảng Tra Sự Cố

| Triệu chứng | Nguyên nhân | Cách sửa |
|---|---|---|
| Terminal in đỏ `!!! ESP32 ESTOP: <lý do>` (xe không chạy dù có lệnh) | ESP32 tự dừng khẩn: `Banh ket / mat encoder` (hết pin, mất nguồn động cơ 12 V, bánh kẹt, encoder đứt) hoặc `Banh quay nguoc chieu` (đấu ngược dây / nạp nhầm `#define XE`) | sửa nguyên nhân rồi gỡ: `ros2 topic pub --once /agvN/estop_reset std_msgs/msg/Empty` hoặc tắt/bật nguồn xe. Cần firmware có `/diag` |
| **Xe thật: mọi thứ treo, không lỗi gì** | sót `use_sim_time:=false` ở một terminal | thêm vào; node đó đang chờ `/clock` mãi |
| `package 'micro_ros_agent' not found` | chưa source workspace agent | `source ~/microros_ws/install/setup.bash` |
| Các node không thấy nhau, `ros2 topic list` thiếu topic | thiếu biến discovery | `export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` |
| RViz / Gazebo cực chậm, cửa sổ trắng như treo | WSL vẽ bằng CPU (llvmpipe) thay vì GPU | `export GALLIUM_DRIVER=d3d12` (đã có trong `~/.bashrc`), mở terminal mới rồi chạy lại |
| RViz in đỏ `GLSL link result ... indexed_8bit_image ... active samplers with a different type` | shader bản đồ của RViz, Mesa kiểm tra khắt khe | `sudo tools/fix_rviz_shader.sh` (một lần; `undo` để trả lại; chạy lại sau khi apt nâng cấp RViz) |
| Tiêu đề cửa sổ Linux có `[WARN:COPY MODE]` | WSLg không mở được bộ nhớ chung (`/mnt/wslg/weston.log`: `rdp_allocate_shared_memory ... Input/output error`), chép hình từng khung — vẫn chạy | chỉ là chữ: file `C:\Users\LRGION\.wslgconfig` có `[system-distro-env]` + `WESTON_RDP_COPY_WARNING_TITLE=false`, rồi `wsl --shutdown` |
| Nav2 mô phỏng đứng im | `/clock` không chạy | bấm **Play** trong Gazebo |
| Nhập đích xong xe không chạy | Nav2 chưa xong | `ros2 run tf2_ros tf2_echo map base_footprint` báo `Invalid frame ID "map"` → đợi thêm |
| ESP32 không kết nối agent sau khi bật lại | ESP32 cần thêm thời gian, hoặc sai `AGENT_IP` | chờ 30 s; kiểm `hostname -I` khớp `AGENT_IP` |
| ESP32 reset liên tục, Serial báo `SUT AP (brownout)` | sụt áp | tách nguồn động cơ khỏi nguồn ESP32, tụ 1000 µF |
| Bringup báo `IMU khong gui goi hop le`; xe quay mà RViz không quay | BNO055 trả toàn số 0 (sau khi chỉ bấm reset ESP32) | **tắt hẳn nguồn xe** rồi bật lại, để xe đứng yên lúc khởi động; vẫn bị thì kiểm 4 dây I2C |
| Xe chỉ chạy hết tốc hoặc dừng | jumper ENA/ENB trên L298N còn nguyên | tháo ra |
| `Connecting....` rồi `Wrong boot mode detected (0x13)` | bo không tự vào chế độ nạp | **giữ nút BOOT** lúc `Connecting....` |
| Tay cầm không tác dụng | `joy_node` đang chạy | thêm `start_joy_node:=false` |
| Đẩy cần lên xe chạy lùi, đẩy trái xe rẽ phải | đảo chiều động cơ | `python3 tools/check_direction.py`, làm theo kết luận; đảo động cơ thì đảo **cả bốn** cờ `INVERT_MOTOR_*`/`INVERT_ENC_*` |
| Xe quay chậm hơn lệnh, xoay lệch > 3° | motor yếu trên sàn | firmware đã tăng Kf 3.0, tích phân ≤ 200; kiểm pin (3S đầy 12.6 V) |
| Trên RViz xe bám cạnh nhưng ngoài sàn lệch vạch | sa bàn chưa khớp vạch thật | [căn sa bàn](#căn-sa-bàn-cho-khớp-vạch-gạch-sau-khi-vẽ-bản-đồ-mới) |
| RViz không hiện hình xe | display RobotModel bị tắt | tích vào ô đó; lưu cấu hình vào `src/my_bot_bringup/rviz/` |
| RViz: `Frame [map] does not exist` | thiếu frame `odom` / lệch QoS `/tf` | `python3 tools/check_tf.py` phải in `odom -> lidar_link OK`; `python3 tools/check_qos.py` |
| `ros2 topic echo` báo không có publisher dù topic tồn tại | lỗi discovery của CLI `ros2` trong WSL | dùng `python3 tools/check_esp32_rates.py` |
| Bản đồ SLAM lệch khỏi tường sau khi quay | (đã sửa trong firmware: stamp `/scan` giữa vòng quét) | `time_increment` của `/scan` phải bằng 0 |
| Trục đỏ của xe không hướng về phía trước thật | LiDAR gắn lệch góc | đặt vật trước mũi xe 0.4–1 m, `python3 tools/check_lidar_offset.py` |
| Bản đồ ra từng mảng rời, gần như không có tường | ảnh quét lật gương | `python3 tools/check_lidar_angle.py`, cho xe quay tại chỗ |
| Bản đồ đầy chấm đen nhỏ | tia rác của LiDAR | đã chỉnh `config/slam_async.yaml` (`occupancy_threshold` 0.3…) |
| Phần lớn bản đồ vẫn xám | tia thưa ở xa | đã chỉnh `slam_async.yaml`; lái chậm, dừng nhiều chỗ |
| Hai xe (mô phỏng): Nav2 một xe không lên | lifecycle mất trả lời khi khởi động | `fleet_sim.sh` tự bật lại tối đa 3 lần; vẫn lỗi thì `down` rồi `up` |
| Hai xe (mô phỏng): Nav2 liên tục báo thất bại | còn sót agent cũ chạy song song | `tools/fleet_sim.sh down` (tắt mọi tiến trình mô phỏng) rồi `up` |

Chi tiết các lỗi firmware đã gặp (stamp, `spin_some`, đồng hồ 1 giây…): [Phụ lục G](#g-các-lỗi-firmware-đã-sửa).

---

# 📚 Phụ Lục

## A. Nối Dây

Sơ đồ đầy đủ mở bằng trình duyệt: [`so_do_day_agv.html`](so_do_day_agv.html). Chân
theo **mạch in 2026-10-05**.

| ESP32 | Nối tới | Ghi chú |
|---|---|---|
| GPIO 25 | L298N **ENA** | PWM bánh trái |
| GPIO 27 / 26 | L298N **IN1 / IN2** | chiều bánh trái |
| GPIO 14 | L298N **ENB** | PWM bánh phải |
| GPIO 33 / 32 | L298N **IN3 / IN4** | chiều bánh phải |
| GPIO 23 / 18 | Encoder trái **A / B** (A_T, B_T) | kéo lên nội |
| GPIO 19 / 17 | Encoder phải **A / B** (A_P, B_P) | kéo lên nội |
| GPIO 21 / 22 | BNO055 **SDA / SCL** | địa chỉ `0x29` (`0x28` nếu ADR nối GND) |
| GPIO 16 | LiDAR **Tx** | UART2, 115200 |
| GPIO 4 | LiDAR **PWM** | tốc độ mô-tơ quét |
| 3V3 | VCC encoder, BNO055 VIN | **không** dùng 5 V cho encoder |
| VIN (5V) | ra của mạch hạ áp | không cắm USB cùng lúc |

Bánh trái → **OUT1/OUT2**, bánh phải → **OUT3/OUT4**. (**Xe 2** đang nối chéo: kênh
27/26/25 chạy bánh phải, 33/32/14 chạy bánh trái — firmware `#define XE 2` đã đổi chân
cho khớp, không cần đổi dây. Xe 1 dùng chân encoder 23/34/4/35, PWM LiDAR 18.) LiDAR VCC lấy 5 V từ mạch hạ áp.
GPIO 13 còn trống (dùng cho chân EN nếu đổi sang LiDAR X4).

> [!WARNING]
> - **Tháo cả hai jumper ENA và ENB** trên L298N, không thì PWM vô tác dụng.
> - **Encoder cấp 3V3, KHÔNG phải 5V** — 5V làm chết GPIO.
> - Không nuôi ESP32 từ chân +5V của L298N. Tụ 1000 µF sát cọc 12 V. Mọi GND về một điểm.

**Nguồn:** pin Li-ion 3S (đầy 12.6 V) → L298N +12V (động cơ) và → mạch hạ áp 5 V →
ESP32 VIN + LiDAR.

## B. Kiểm Tra Phần Cứng Lần Đầu

Mở Serial Monitor 115200, làm lần lượt:

| # | Việc | Đúng khi |
|---|---|---|
| 1 | Nhấc xe, quay tay bánh trái đúng 10 vòng | `count` đổi ~21500. Quay tiến mà RPM âm → đảo `INVERT_ENC_L` |
| 2 | Bơm lệnh tiến (lệnh dưới, **kê bánh lên trước**) | cả hai bánh quay tiến; bánh nào lùi → đảo `INVERT_MOTOR_x` |
| 3 | Xoay xe 90° trong ~1 giây | `gyroZ` ≈ 1.57 |
| 4 | Đọc log | `scan 8.5 Hz`, vài trăm điểm/vòng |
| 5 | Dòng chẩn đoán | `pub/s: scan 8 odom 50 imu 50` |

```bash
cd ~/robot_ws_3
source install/setup.bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

Kiểm dữ liệu lên đủ (Terminal 1 của Kịch bản 2 trong HUONG_DAN_CHAY.md đang chạy):
```bash
python3 tools/check_qos.py          # QoS có khớp không
python3 tools/check_tf.py           # phải in odom -> lidar_link OK
python3 tools/check_esp32_rates.py  # /scan ~8.7 Hz, /odom /imu/data ~50 Hz
```

**Dòng chẩn đoán firmware** (dòng thứ hai mỗi giây):

| Cột | Bình thường | Lệch thì |
|---|---|---|
| `ros_task …/s` | 500–700 | thấp → vòng lặp bị chặn |
| `pub/s scan` / `odom` / `imu` | 8–9 / 50 / 50 | thấp → xem `ca_vong` |
| `spin` | < 2000 | ~980000 → `spin_some` chặn |
| `imu` | 5000–9000 | lớn → dây I2C dài/nhiễu |
| `sync` | < 300 | ~600000 **bình thường khi xe đứng yên** (ping agent) |
| `ca_vong` | 8000–12000 | 750000 → lệnh chặn trong vòng lặp |
| `loi pub` | 0 | > 0 → vượt MTU / hết bộ đệm |

## C. Đo Định Vị Kỹ

| Kết quả `check_scan_fit.py` | Nghĩa |
|---|---|
| lệch trung vị < 0.05 m, amcl ± vài cm | tốt |
| lệch nhỏ lúc đứng, lớn lúc chạy | trễ giữa hai lần AMCL sửa; dùng được |
| amcl ± nhỏ nhưng lệch > 0.15 m | AMCL tự tin mà sai — đặt lại vị trí |
| amcl ± lớn mãi | vị trí ban đầu sai quá xa / phòng đổi nhiều |

Đo từng ảnh quét khi xe chạy:
```bash
ros2 bag record -o ~/bags/thu1 /scan /joint_states /imu/data /tf /tf_static /odom /amcl_pose   # Ctrl-C khi xong
python3 tools/eval_bag_localization.py ~/bags/thu1 src/my_bot_bringup/config/real_room_map.yaml
```
**Đạt:** sai vị trí lúc chạy trung vị < 5 cm, p90 < 10 cm. Xe này đo được
**3.6 / 6.1 / 10.4 cm** (trung vị / p90 / max), sai hướng tối đa ~3°.

## D. Vì Sao Cần Các Dòng Trong `~/.bashrc`

- **`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`** — mạng WSL làm discovery mặc định thất
  bại: các node **vẫn chạy nhưng không thấy nhau**, không có lỗi nào.
- **`source ~/microros_ws/install/setup.bash`** — `micro_ros_agent` không có trong apt,
  đã build sẵn ở `~/microros_ws`. Thiếu dòng này: `package 'micro_ros_agent' not found`.
- **`fix_wsl_clock.sh`** — `systemd-timesyncd` cãi nhau với Hyper-V làm đồng hồ Ubuntu
  chậm 8.3% rồi giật 2.4 s mỗi ~35 s; tay cầm bị ngắt từng cơn.
- **`GALLIUM_DRIVER=d3d12`** — Mesa trên WSL mặc định chọn llvmpipe (vẽ bằng CPU, RViz
  ~150% CPU, cửa sổ như treo). Ép d3d12 thì dùng card AMD qua DirectX: RViz còn ~16% CPU.

## E. Thuật Toán Đang Dùng

| Tầng | Thuật toán | Cấu hình |
|---|---|---|
| Tìm đường (`grid_navigator.py`) | Dijkstra trên lưới 5×5 nối 4 hướng: **ít cạnh nhất**, bằng nhau thì **ít chỗ rẽ nhất**; né điểm/cạnh bị chặn. Tuyến cắt thành **đoạn thẳng trên cạnh** (cạnh liền nhau cùng hướng gộp một đoạn), giao thẳng cho controller qua action `follow_path` | `grid_real.yaml` |
| Xoay tại chỗ (`grid_navigator.py`) | vòng kín theo **gyro** `/imu/data`; dừng lệnh khi góc còn lại ≤ `coast` × ω (0.36 s, đo trên xe); giữ lệnh ≥ 0.15 s; lệch > 3° thì xoay bù (tối đa 5 lần), vẫn chưa được thì báo `CHƯA ĐẠT` | `rotate:` trong `grid_real.yaml` |
| Bám đoạn thẳng (Nav2) | **Regulated Pure Pursuit** 0.15 m/s, nhìn trước 0.2 m; dừng trong 2 cm quanh điểm lưới | `FollowPath`, `navigation.yaml` |
| An toàn | `velocity_smoother` (giới hạn gia tốc) → `collision_monitor` (giảm/dừng nếu sẽ va trong 1.2 s) | `navigation.yaml` |
| Bánh xe (ESP32) | PID tốc độ từng bánh 50 Hz, feedforward Kf 3.0 | firmware |

Xe thật (2026-10-03): 9/9 lần xoay ≤ 2.8°, 4–6 s mỗi lần; tới đích lệch 0.4–1.6 cm;
lúc chạy lệch cạnh trung bình ≤ 0.6 cm (theo AMCL).

**Ai làm gì:** ESP32 đọc LiDAR → `/scan`, encoder → `/joint_states` (**góc tích luỹ**,
mất gói không mất quãng đường), BNO055 → `/imu/data`, chạy PID và nhận `/cmd_vel`.
Laptop: `wheel_odometry.py` → `/odom` + TF, AMCL, Nav2, sa bàn. Bán kính lăn bánh chỉnh
ở laptop, không cần nạp chip:
```bash
ros2 launch my_bot_bringup bringup_real.launch.py wheel_radius:=0.0328
```
Nguồn góc quay `yaw_source`: `imu_quaternion` (mặc định) · `imu_gyro` · `wheels`.

## F. Quy Tắc Điều Phối Hai Xe

```
traffic_manager.py ──/fleet/agv1/cmd──▶ fleet_agent.py (/agv1) ──▶ Nav2 /agv1 ──▶ xe 1
  lập đường, giữ chỗ,  ◀──/fleet/status──  chạy tới ô được phép
  gỡ kẹt               ──/fleet/agv2/cmd──▶ fleet_agent.py (/agv2) ──▶ Nav2 /agv2 ──▶ xe 2
```

1. Hai xe không cùng đích. Được nhận ô xe kia đang **đỗ** làm đích: xe đỗ né đi ngay.
2. Đường ít cạnh, ít rẽ; đi trên đường còn lại của xe kia bị cộng chi phí (+2/ô), đi
   ngược chiều xe kia trên cùng cạnh +6. Xe đang đỗ là vật cản cứng.
3. Xe chiếm mọi ô cách nó < nửa ô + bán kính xe; chỉ được đi tới ô đã **giữ chỗ**
   (xem trước 2 ô).
4. Hai xe xin cùng một ô: **xe gần đích hơn** được trước; bằng nhau thì xe 1.
5. Chờ > 3 s vì bị chặn: xe kia **đỗ / mất liên lạc** → đi vòng, không được thì xe đỗ
   né sang ô trống; **chờ nhau** hoặc xe kia **đứng yên hẳn** → xe xa đích hơn nhường
   (đi vòng → né sang ô trống rồi quay lại → đổi vai → dừng cả hai, báo lỗi). Xe kia
   đang chạy / đang xoay thì cứ chờ.
6. Mất liên lạc: agent không nhận lệnh 1 s → tự dừng; bộ điều phối không nhận trạng thái
   1 s → giữ nguyên các ô xe đó giữ.
7. Vật cản thật / Nav2 thất bại → lập lại đường từ chỗ xe dừng (tối đa 5 lần).

**Đã kiểm trên mô phỏng (2026-10-05):** hai xe không lúc nào gần nhau dưới 57 cm, tới
đích lệch ≤ 2 cm, xoay ≤ 2.3°.

| Kịch bản thử | Diễn biến |
|---|---|
| Đổi chỗ trên hàng 3 | xe đỗ ở đích né ngay; xe kia đi vòng hàng 4 |
| Cắt ngang ở (3,3) | một xe dừng chờ xe kia qua rồi đi |
| Đối đầu | kẹt → xe xa đích hơn né sang ô bên, xe kia qua rồi đi tiếp |
| Đích là chỗ xe kia đỗ | xe đỗ né sang ô bên |
| Ưu tiên gần đích | xe còn ít ô hơn vào trước |
| Tắt bộ điều phối | hai xe tự dừng sau ~2 s |
| Tắt agent một xe | báo mất liên lạc 1 s; xe kia đi vòng; bật lại thì chạy nốt |
| Vật cản bất ngờ | dừng < 1 s, đi vòng |

Mô phỏng dùng bản sao `grid_real.yaml` với `coast: 0` (xe ảo không ma sát). **Chưa
chạy trên hai xe thật** — khi có xe thứ hai: mỗi xe một ESP32 `ROBOT_ID` riêng, AMCL +
Nav2 riêng trong namespace, chung bản đồ.

## G. Các Lỗi Firmware Đã Sửa

| Lỗi | Triệu chứng | Đã sửa |
|---|---|---|
| `rclc_executor_spin_some` chặn tới khi có dữ liệu | mọi topic ~1 Hz | truyền timeout `0` |
| `rmw_uros_epoch_nanos()` chỉ phân giải 1 giây | stamp nhảy đúng 1000 ms, Nav2 báo `message too old` | neo epoch một lần rồi chạy bằng `esp_timer` |
| `sync_session` lệch tới 1 giây | cả cụm stamp trễ ~1 s | chờ đúng cạnh tick rồi mới sync |
| stamp `/scan` gắn lúc gửi | bản đồ lệch sau khi quay | stamp lùi về giữa vòng quét, `time_increment = 0` |
| `/tf` best-effort | `Frame [map] does not exist` | `/tf`, `/odom` đổi sang reliable |
| Wi-Fi rơi gói → im 1 s | ảnh quét trượt 10–25 cm | odometry dùng giá trị tích luỹ qua lỗ hổng (giữ `/joint_states` RELIABLE: best-effort làm lệnh trễ 2.9 s) |
| PID kẹp tích phân 80, Kf đo không tải | bánh chỉ đạt 55–66% lệnh trên sàn | Kf 3.0, tích phân ≤ 200 |

**Còn mở:** cột `sync` khi đang lái chưa xác nhận im lặng; còn vài khoảng trống > 200 ms
lúc xe đứng yên chưa rõ nguyên nhân.

## H. Các Hướng Dẫn Khác

### H1. Chuẩn Bị Máy Mới (chỉ làm MỘT lần cho mỗi máy)

> [!WARNING]
> Chạy lại bước này sẽ thêm dòng trùng vào `~/.bashrc`. Kiểm tra đã làm chưa:
> `grep -c DISCOVERY ~/.bashrc` in ra `1` là đã xong — bỏ qua.

Mở **1 terminal WSL**:
```bash
# 1. Tự source ROS 2, micro-ROS agent và dự án ở mọi terminal mới
echo 'source /opt/ros/jazzy/setup.bash'                >> ~/.bashrc
echo 'source ~/microros_ws/install/setup.bash'         >> ~/.bashrc
echo 'source ~/robot_ws_3/install/setup.bash'          >> ~/.bashrc

# 2. Sửa lỗi discovery của WSL (BẮT BUỘC)
echo 'export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST'  >> ~/.bashrc

# 2b. RViz/Gazebo vẽ bằng GPU (mặc định WSL vẽ bằng CPU -> chậm như treo)
echo 'export GALLIUM_DRIVER=d3d12'                   >> ~/.bashrc
source ~/.bashrc

# 3. Build dự án
cd ~/robot_ws_3
colcon build --symlink-install

# 4. Sửa đồng hồ WSL (cần mật khẩu) - dòng "Sau:" phải in tick=10000
sudo bash ~/robot_ws_3/tools/fix_wsl_clock.sh
```

> [!TIP]
> **Khi nào cần build lại?** Sau khi sửa file trong `src/` (launch, cấu hình,
> bản đồ mới). Các script `.py` ở thư mục gốc (`grid_navigator.py`,
> `traffic_manager.py`…) **không cần build**.
>
> `colcon build` luôn chạy trong `~/robot_ws_3`. Chạy ở `~` thì nó quét cả các
> bản sao cũ (`robot_ws`, `robot_ws_2`…) và báo `Duplicate package names`.

Vì sao cần mấy dòng đó: xem [Phụ lục D](#d-vì-sao-cần-các-dòng-trong-bashrc).

### H2. Mô Phỏng Gazebo — Lưới 5×5 (Cần 3 Terminals)

Thử thuật toán tìm đường và né vật cản mà không cần xe thật. Lưới mô phỏng ô 2 m
trong phòng 10×10 m.

#### Terminal 1: Bật Gazebo
```bash
cd ~/robot_ws_3
source install/setup.bash
ros2 launch my_bot_bringup gazebo.launch.py
```
Chờ cửa sổ Gazebo hiện căn phòng 4 tường xám và xe hình trụ màu cam.

> [!IMPORTANT]
> Bấm **▶ Play** ở góc dưới-trái nếu mô phỏng đang dừng. Không bấm thì `/clock`
> không chạy và Terminal 2 đứng im. Không cần giao diện: thêm `headless:=true`.

#### Terminal 2: Bật Nav2
```bash
cd ~/robot_ws_3
source install/setup.bash
ros2 launch my_bot_bringup navigation.launch.py
```
Chờ log ngừng trôi. Xe tự định vị ở `(0, 0)`, **không cần** bấm "2D Pose Estimate".

#### Terminal 3: Nhập Đích
```bash
cd ~/robot_ws_3
source install/setup.bash
python3 grid_navigator.py
```
Dòng đầu phải in `Che do: MO PHONG (use_sim_time=true)`. Nhập **hàng** `1`–`5` ↵,
**cột** `1`–`5` ↵, rồi **hướng tại đích** (`D`/`T`/`B`/`N`, Enter = giữ hướng).
Gõ `q` để thoát.

#### Terminal 4 (tuỳ chọn): RViz
```bash
cd ~/robot_ws_3
source install/setup.bash
rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz --ros-args -p use_sim_time:=true
```
Để thấy lưới: **Add** → tab **By topic** → **`/grid_visualization`** → **MarkerArray** → OK.

> [!TIP]
> **Thử né vật cản:** trong Gazebo bấm biểu tượng **hình hộp** rồi bấm xuống sàn
> để thả hộp chắn ngang đường xe. Terminal 3 in
> `Phát hiện vật cản trên đường đi! ... Đang tính toán đường vòng...` rồi xe đi
> đường khác. Đặt hộp đúng giữa hai điểm lưới để thấy cạnh bị chặn.

### H3. Nạp Firmware Xe Thật

Làm lần đầu và mỗi khi firmware thay đổi. Dùng **Arduino IDE** trên Windows.

**Bước 1 — Mở sketch:** `D:\CTU\Ardunio ide\code\Xe_lidar\Xe_lidar.ino` (bản sao của
`firmware/esp32_agv_base/esp32_agv_base.ino`).

**Bước 2 — Chọn partition:** **Tools → Partition Scheme → Huge APP (3MB No OTA/1MB SPIFFS)**.

**Bước 3 — Wi-Fi (chỉ lần đầu mỗi máy):** chép `wifi_config.example.h` thành
`wifi_config.h` **cùng thư mục với file `.ino`**, rồi điền:
```cpp
#define WIFI_SSID   "..."             // Wi-Fi của bạn
#define WIFI_PASS   "..."
#define AGENT_IP    "192.168.1.21"    // IP laptop - xem bằng: hostname -I
```
> [!NOTE]
> `wifi_config.h` **không nằm trong git** — mật khẩu không bao giờ lên GitHub.
> Lần nạp sau chỉ cần chép đè file `.ino`, để nguyên `wifi_config.h`.

**Bước 3b — Chọn xe (MỖI LẦN NẠP):** gần đầu phần `1. CAU HINH` có dòng
`#define XE 2`. Đổi cho đúng xe đang cắm USB:

| `XE` | Xe | Khác nhau |
|---|---|---|
| `1` | xe 1 — mạch cũ | encoder 23/34/4/35, PWM LiDAR 18, LiDAR bù 180°, khoảng cách ×1.0 |
| `2` | xe 2 — mạch in 2026-10-05 | encoder 23/18/19/17, PWM LiDAR 4, kéo lên, LiDAR bù 180°, khoảng cách **×1.062** (LiDAR xe này đo thiếu ~6%) |

Nạp nhầm số xe: encoder không đếm (sai chân) hoặc AMCL không khớp được bản đồ.
Sau khi nạp, `ros2 node list` phải thấy `/agv<số xe>/esp32_agv_base_<số xe>`: mọi topic
của xe nằm trong `/agv<số xe>/` (micro-ROS gắn ID xe), nên chạy một xe thì
`bringup_real.launch.py xe:=<số xe>`, hai xe thì `tools/fleet_real.sh up` (Kịch bản 4).

**Bước 4 — Nạp:** bấm Upload. Khi IDE hiện `Connecting....` thì **giữ nút BOOT**
trên ESP32 tới khi thanh tiến trình chạy.

> [!WARNING]
> Không giữ BOOT sẽ báo `Wrong boot mode detected (0x13)`.

**Bước 5 — Kiểm tra:** mở **Serial Monitor 115200**, nhấn EN. Phải thấy:
```
>>> Reset: CAP NGUON            <- nếu là SUT AP (brownout) thì lỗi nguồn
Wi-Fi OK, IP xe: 192.168.1.150
Da thay agent!                  <- chưa thấy = agent (Terminal 1 trong HUONG_DAN_CHAY.md) chưa chạy
Dong ho: tick1 ... sync -> 0 ...
LiDAR start(): OK
```

> [!IMPORTANT]
> **Khởi động lại xe = tắt hẳn nguồn vài giây rồi bật**, để xe đứng yên 2 giây đầu
> (BNO055 đo độ lệch gyro). Chỉ bấm nút reset ESP32 có lúc làm BNO055 trả toàn số 0.

Sơ đồ nối dây và bảng chân: [Phụ lục A](#a-nối-dây) và [`so_do_day_agv.html`](so_do_day_agv.html).
Kiểm tra phần cứng lần đầu: [Phụ lục B](#b-kiểm-tra-phần-cứng-lần-đầu).

### H4. Kiểm Tra Định Vị Trên Bản Đồ — AMCL (Cần 4 Terminals)

Kiểm tra LiDAR có bám đúng bản đồ **trước** khi cho xe tự chạy. Kịch bản này không có
bộ điều khiển nào phát lệnh, nên lái tay thoải mái.

> [!IMPORTANT]
> **Tắt SLAM trước** (Ctrl-C terminal SLAM của Kịch bản 1 trong HUONG_DAN_CHAY.md). SLAM và AMCL cùng phát
> `map → odom`; chạy chung thì xe nhảy qua lại trong RViz.

#### Terminal 1: Cầu Nối ESP32
```bash
cd ~/robot_ws_3
source install/setup.bash
ros2 launch my_bot_bringup bringup_real.launch.py
```

#### Terminal 2: Bản Đồ + AMCL
```bash
cd ~/robot_ws_3
source install/setup.bash
ros2 launch my_bot_bringup localization.launch.py use_sim_time:=false
```
Chờ `Managed nodes are active`, rồi **đặt vị trí ban đầu tự động** (xe đứng yên ở đâu
cũng được):
```bash
cd ~/robot_ws_3
source install/setup.bash
python3 tools/auto_initial_pose.py
```

#### Terminal 3: Lái Tay
Tay cầm như Kịch bản 1 trong HUONG_DAN_CHAY.md, hoặc bàn phím ([H5](#h5-lái-xe-bằng-bàn-phím-thay-cho-tay-cầm)).

#### Terminal 4: RViz
```bash
cd ~/robot_ws_3
source install/setup.bash
rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz
```
Fixed Frame = `map`. **Điểm quét đỏ phải trùng tường đen.** Lệch thì bấm
**2D Pose Estimate**, bấm vào chỗ xe thật rồi kéo theo hướng đầu xe.

#### Đo Độ Khớp (tuỳ chọn, Terminal 5)
```bash
cd ~/robot_ws_3
source install/setup.bash
python3 tools/check_scan_fit.py --duration 90
```
Lái chậm khắp phòng. **Đạt:** lệch trung vị < 5 cm. Bảng đọc kết quả và cách đo kỹ
hơn: [Phụ lục C](#c-đo-định-vị-kỹ).

> [!TIP]
> AMCL chỉ sửa vị trí khi xe đã chạy ≥ 0.1 m hoặc quay ≥ 6°. Đứng yên thì đám mũi
> tên xanh không co lại — không phải lỗi. Cho xe quay tại chỗ một vòng để nó chốt.

Xong thì **Ctrl-C Terminal 2** trước khi sang Kịch bản 2 trong HUONG_DAN_CHAY.md.

### H5. Lái Xe Bằng Bàn Phím (thay cho tay cầm)

Dùng thay Terminal tay cầm ở Kịch bản 1 của HUONG_DAN_CHAY.md. Giữ con trỏ trong terminal này:
```bash
cd ~/robot_ws_3
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
`i` tiến · `,` lùi · `j` trái · `l` phải · `k` dừng · `z`/`x` giảm/tăng tốc.

