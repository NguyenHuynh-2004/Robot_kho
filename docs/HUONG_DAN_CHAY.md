# Hướng dẫn chạy AGV điều hướng lưới 5×5

Xe tự hành trong phòng 10×10m. Bạn nhập toạ độ lưới, xe tự tìm đường bằng BFS
qua Nav2 và tự né vật cản động.

Có **hai chế độ**:

| Chế độ | Dùng khi | Số terminal |
|---|---|---|
| [A — Mô phỏng](#chế-độ-a--mô-phỏng-gazebo) | thử thuật toán, không cần phần cứng | 3 (+1 tuỳ chọn) |
| [B — Xe thật](#chế-độ-b--xe-thật) | chạy trên xe ESP32 | 3–5 |

---

## Bước 0 — Chuẩn bị một lần duy nhất

Mở terminal WSL:

```bash
cd ~/robot_ws_3

# 1. Các dòng source bắt buộc
echo 'source /opt/ros/jazzy/setup.bash'                >> ~/.bashrc
echo 'source ~/microros_ws/install/setup.bash'         >> ~/.bashrc
echo 'source ~/robot_ws_3/install/setup.bash'          >> ~/.bashrc

# 2. Sửa lỗi discovery của WSL (BẮT BUỘC, xem giải thích bên dưới)
echo 'export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST'  >> ~/.bashrc

source ~/.bashrc
colcon build --symlink-install
```

Sau bước này, mỗi terminal mới chỉ cần `cd ~/robot_ws_3`.

### Vì sao cần 2 dòng đặc biệt đó

**`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`** — kernel WSL để `eth0` ở trạng
thái khiến multicast discovery mặc định thất bại. Không có dòng này, các node
**vẫn chạy nhưng không nhìn thấy nhau**: Nav2 sẽ không thấy Gazebo, và không
có thông báo lỗi nào. Đã kiểm chứng: talker publish bình thường mà
`ros2 topic list` không thấy topic.

**`source ~/microros_ws/install/setup.bash`** — `micro_ros_agent` **không có
trong apt** (gói `ros-jazzy-micro-ros-agent` không tồn tại). Trên máy này nó
đã được build sẵn ở `~/microros_ws`. Thiếu dòng này sẽ gặp:

```
[ERROR] [launch]: package 'micro_ros_agent' not found
```

---

# CHẾ ĐỘ A — Mô phỏng (Gazebo)

Mở lần lượt, **terminal trước khởi động xong mới mở terminal sau**.

### Terminal 1 — Gazebo

```bash
ros2 launch my_bot_bringup gazebo.launch.py
```

Chờ tới khi cửa sổ Gazebo hiện căn phòng 4 tường xám và xe hình trụ màu cam.

> Bấm **▶ Play** ở góc dưới-trái nếu mô phỏng đang tạm dừng. Không bấm thì
> `/clock` không chạy và Terminal 2 sẽ đứng im.

Không cần giao diện thì thêm `headless:=true`.

### Terminal 2 — Nav2

```bash
ros2 launch my_bot_bringup navigation.launch.py
```

Chờ log ngừng trôi và thấy `Creating bond timer...` cho `amcl`,
`controller_server`, `planner_server`, `bt_navigator`.

Kiểm tra xe đã tự định vị:

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

Phải in `Translation: [0.000, 0.000, 0.000]` liên tục. **Không cần** bấm
"2D Pose Estimate" — tham số `set_initial_pose` trong `navigation.yaml` lo việc đó.

### Terminal 3 — Điều khiển

```bash
cd ~/robot_ws_3 && python3 grid_navigator.py
```

Dòng đầu phải in `Che do: MO PHONG (use_sim_time=true)`.

Nhập **hàng** `0`–`4` ↵ rồi **cột** `0`–`4` ↵. Gõ `q` để thoát.

### Terminal 4 (tuỳ chọn) — RViz

```bash
rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz \
  --ros-args -p use_sim_time:=true
```

Để thấy lưới 5×5: **`Add`** → tab **`By topic`** → **`/grid_visualization`**
→ **`MarkerArray`** → OK.

### Thử né vật cản

Trong cửa sổ Gazebo, bấm biểu tượng **hình hộp** trên thanh công cụ rồi bấm
xuống sàn để thả hộp **chắn ngang đường xe đang đi**. Terminal 3 sẽ in:

```
[WARN] Phát hiện vật cản trên đường đi! Đang tiến hành dừng xe khẩn cấp...
[INFO] Đã dừng xe thành công. Đang tính toán đường vòng...
[INFO] Bắt đầu Replan từ (2, 3) đến (2, 0)
[INFO] Lộ trình (8 bước) đã tạo!
```

Xe có bán kính 0.6m, node cách nhau 2m — đặt hộp **đúng giữa hai node** để
thấy cơ chế chặn cạnh.

---

# CHẾ ĐỘ B — Xe thật

## B1. Nạp firmware

Mở `firmware/esp32_agv_base/esp32_agv_base.ino` trong Arduino IDE.

**Tools → Partition Scheme → Huge APP (3MB No OTA/1MB SPIFFS)** ← bắt buộc.
Partition mặc định chỉ còn 19% dư, thêm chút nữa là tràn flash.

Sửa các giá trị đầu file:

```cpp
#define WIFI_SSID   "..."             // wifi của bạn
#define WIFI_PASS   "..."
#define AGENT_IP    "192.168.1.21"    // IP laptop; xem bằng: hostname -I

#define WHEEL_RADIUS_M      0.0325f   // ĐO: bán kính bánh LÚC CÓ TẢI
#define WHEEL_SEPARATION_M  0.190f    // ĐO: khoảng cách tâm–tâm hai bánh
```

> Hai giá trị `WHEEL_*` hiện là số đặt tạm. Sai hai số này thì odometry sai và
> Nav2 vô dụng. Quy trình hiệu chỉnh nằm ở **Phụ lục A** trong file firmware.

## B2. Nối dây — 14 chân

### L298N (động cơ)

| ESP32 | → L298N |
|---|---|
| GPIO 25 | **ENA** (PWM bánh trái) |
| GPIO 27 | **IN1** |
| GPIO 26 | **IN2** |
| GPIO 14 | **ENB** (PWM bánh phải) |
| GPIO 33 | **IN3** |
| GPIO 32 | **IN4** |
| GND | **GND** |

Bánh trái → **OUT1/OUT2**, bánh phải → **OUT3/OUT4**.

> ⚠️ **Tháo cả hai jumper ENA và ENB.** Còn jumper thì hai chân đó bị chốt
> HIGH, PWM vô tác dụng, xe chỉ chạy full tốc hoặc dừng.

### Encoder (2 kênh)

| ESP32 | → Encoder | Dây |
|---|---|---|
| GPIO 23 | **A** bánh trái | vàng |
| GPIO 34 | **B** bánh trái | xanh lá |
| GPIO 4 | **A** bánh phải | vàng |
| GPIO 35 | **B** bánh phải | xanh lá |
| **3V3** | VCC cả hai | |
| GND | GND cả hai | |

> ⚠️ **Cấp encoder từ 3V3, KHÔNG phải 5V.** Encoder xuất tín hiệu ở mức bằng
> điện áp nguồn của nó; 5V vượt ngưỡng 3.3V và làm chết cổng GPIO.

GPIO 34/35 chỉ vào được và không có điện trở kéo lên nội bộ. Encoder Hall
push-pull chạy tốt; nếu là loại hở cực thu thì hàn thêm 10kΩ lên 3V3.

### BNO055 (IMU)

| ESP32 | → BNO055 |
|---|---|
| GPIO 21 | **SDA** |
| GPIO 22 | **SCL** |
| 3V3 | **VIN** |
| GND | **GND** |

Địa chỉ I2C mặc định trong firmware là `0x29`. Nếu Serial báo
`khong thay BNO055`, đổi `BNO055_ADDR` thành `0x28`.

### YDLIDAR X2/X2L — 4 chân

| LiDAR | → ESP32 |
|---|---|
| **VCC** | 5V **từ nguồn ngoài**, không lấy từ ESP32 |
| **GND** | GND chung |
| **Tx** | **GPIO 16** |
| **PWM** | **GPIO 18** |

GPIO 17 và 19 **để trống** — LiDAR này chỉ phát một chiều, không nhận lệnh
(đã kiểm thư viện: `LDS_YDLIDAR_X2_X2L::start()` không ghi byte nào ra UART),
và driver X2/X2L không dùng chân EN.

> Nếu sau này đổi sang **YDLIDAR X4**, driver đó *có* dùng chân EN — phải gán
> `LIDAR_EN_PIN` một GPIO thật (17 hoặc 19 đang rảnh).

### Nguồn

```
Pin 7–12V ──┬── L298N +12V ──── động cơ
            │
            └── mạch hạ áp 5V ──┬── ESP32 (chân 5V/VIN)
               (buck / BEC)     └── LiDAR 5V

TẤT CẢ GND nối chung về MỘT điểm
```

Ba điều bắt buộc:

1. **Không nuôi ESP32 từ chân +5V của L298N.** Bộ ổn áp trên bo đó yếu và
   nhiễu; động cơ khởi động là ESP32 reset.
2. **Tụ 1000µF** sát chân nguồn động cơ của L298N.
3. **Không cắm USB và nguồn ngoài cùng lúc** vào ESP32.

Nếu Serial in `>>> Reset: SUT AP (brownout)` liên tục thì đúng là vấn đề
nguồn, không phải lỗi code.

## B3. Kiểm tra phần cứng — làm trước khi chạy Nav2

Mở **Serial Monitor 115200** và làm lần lượt. Đừng nhảy bước.

| # | Việc | Đúng khi |
|---|---|---|
| 1 | Nhấc xe lên, quay tay bánh trái đúng 10 vòng | `count` đổi ~12000. Quay tiến mà RPM âm → đặt `INVERT_ENC_L = true` |
| 2 | Bơm lệnh tiến (xem lệnh dưới) | **cả hai bánh quay tiến**. Bánh nào lùi → đảo `INVERT_MOTOR_x` |
| 3 | Xoay xe 90° trong ~1 giây | `gyroZ` ≈ **1.57** |
| 4 | Đọc log | `scan 8.5 Hz` và vài trăm điểm/vòng |

Lệnh cho bước 2 (xe **sẽ chạy** — kê bánh lên khỏi mặt đất trước):

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

Cả 4 bước đúng rồi mới hiệu chỉnh `WHEEL_RADIUS_M` / `WHEEL_SEPARATION_M`.

## B4. Vẽ bản đồ phòng thật

Làm trước lần chạy điều hướng đầu tiên. Có hai cách lái.

### Cách 1 — Bàn phím (đơn giản nhất)

**Terminal 1** — cầu nối ESP32:
```bash
ros2 launch my_bot_bringup bringup_real.launch.py
```
Cấp nguồn ESP32. Log agent phải hiện `session established` và `client_key: 0x11111111`.

**Terminal 2** — SLAM:
```bash
ros2 launch my_bot_bringup slam.launch.py use_sim_time:=false
```

**Terminal 3** — lái:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Giữ con trỏ trong terminal này: **`i`** tiến, **`,`** lùi, **`j`** rẽ trái,
**`l`** rẽ phải, **`k`** dừng. **`z`/`x`** giảm/tăng tốc.

**Terminal 4** — xem bản đồ:
```bash
rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz
```

### Cách 2 — Tay cầm (bản đồ mịn hơn nhờ cần analog)

Tay cầm USB **không hiện trong WSL**: kernel WSL không bật
`CONFIG_INPUT_JOYSTICK` nên không có driver joystick nào, kể cả `xpad`. Vì vậy
phải đọc tay cầm bên Windows rồi đẩy qua UDP.

**Trên Windows** (PowerShell hoặc CMD, *không phải* WSL):
```
python \\wsl.localhost\Ubuntu\home\huynh\robot_ws_3\tools\windows_gamepad_bridge.py
```
Giữ cửa sổ này mở. Nó in `re=... tien=... LB=... RB=...` mỗi 2 giây.

**Trong WSL**, thêm 2 terminal vào Cách 1 (thay Terminal 3):

```bash
# T3a: nhận tay cầm -> /joy
cd ~/robot_ws_3 && python3 tools/joy_udp_bridge.py

# T3b: /joy -> /cmd_vel
ros2 launch my_bot_bringup joy_teleop.launch.py start_joy_node:=false
```

> `start_joy_node:=false` **bắt buộc** trên WSL — `joy_node` sẽ không bao giờ
> tìm thấy tay cầm.

Điều khiển: **giữ LB rồi đẩy cần trái**. Thả LB là xe dừng. Giữ thêm **RB** để
chạy nhanh gấp đôi (0.25 → 0.5 m/s).

### Lưu bản đồ

Khi bản đồ trong RViz đã kín hết tường:

```bash
ros2 run nav2_map_server map_saver_cli -f src/my_bot_bringup/config/my_room_map
colcon build --symlink-install
```

## B5. Chạy điều hướng

**Terminal 1** — cầu nối ESP32:
```bash
ros2 launch my_bot_bringup bringup_real.launch.py
```

Kiểm tra dữ liệu lên đủ:
```bash
ros2 topic hz /scan     # ~8.5 Hz
ros2 topic hz /odom     # ~50 Hz
```

**Terminal 2** — Nav2:
```bash
ros2 launch my_bot_bringup navigation.launch.py use_sim_time:=false
```

**Terminal 3** — điều khiển:
```bash
cd ~/robot_ws_3 && python3 grid_navigator.py --real
```
Dòng đầu phải in `Che do: XE THAT (use_sim_time=false)`.

**Terminal 4 (tuỳ chọn)** — RViz, như Chế độ A nhưng bỏ `use_sim_time`.

---

## Bảng tra toạ độ lưới

| | Col 0 | Col 1 | Col 2 | Col 3 | Col 4 |
|---|---|---|---|---|---|
| **Row 4** | (-4, 4) | (-2, 4) | (0, 4) | (2, 4) | (4, 4) |
| **Row 3** | (-4, 2) | (-2, 2) | (0, 2) | (2, 2) | (4, 2) |
| **Row 2** | (-4, 0) | (-2, 0) | **(0,0)** xuất phát | (2, 0) | (4, 0) |
| **Row 1** | (-4,-2) | (-2,-2) | (0,-2) | (2,-2) | (4,-2) |
| **Row 0** | (-4,-4) | (-2,-4) | (0,-4) | (2,-4) | (4,-4) |

Row tăng = đi lên (+Y), Col tăng = sang phải (+X).

---

## Bảng tra sự cố

| Triệu chứng | Nguyên nhân | Cách sửa |
|---|---|---|
| `package 'micro_ros_agent' not found` | chưa source workspace agent | `source ~/microros_ws/install/setup.bash` |
| Các node không thấy nhau, `ros2 topic list` thiếu topic | thiếu biến discovery | `export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` |
| Nav2 đứng im lúc khởi động | `/clock` không chạy | bấm **Play** trong Gazebo |
| Nhập toạ độ xong xe không chạy | Nav2 chưa xong | `tf2_echo map base_footprint` báo `Invalid frame ID "map"` → đợi thêm |
| **Xe thật: mọi thứ treo, không lỗi gì** | sót `use_sim_time:=false` ở một terminal | node đó đợi `/clock` vĩnh viễn và **không in lỗi** |
| ESP32 reset liên tục | sụt áp | Serial báo `SUT AP (brownout)` → chưa tách nguồn động cơ |
| Xe chỉ full tốc hoặc dừng | jumper ENA/ENB còn nguyên | tháo ra |
| Tay cầm không tác dụng | `joy_node` đang chạy | phải thêm `start_joy_node:=false` |
| `ros2 topic echo` báo không có publisher dù topic tồn tại | lỗi discovery của **CLI** `ros2` trong WSL | dùng `ros2 topic hz`, hoặc viết node rclpy để đo |

> Lỗi hay gặp nhất ở Chế độ B là **quên `use_sim_time:=false`**. Node đó sẽ
> chờ `/clock` mãi mãi mà không in bất cứ thông báo nào.

---

## Phạm vi đã kiểm chứng

**Chế độ A** — đã chạy thật nhiều lần. Lần cuối: xe đi từ (2,2) tới (2,0), gặp
vật cản thả giữa đường, dừng khẩn cấp, replan **1 lần**, đi vòng và tới đích
tại (-3.98, 0.24).

**Chế độ B** — ESP32 đã kết nối thật và phát đủ 4 topic với dữ liệu hợp lệ:

| Topic | Frame | Dữ liệu |
|---|---|---|
| `/scan` | `lidar_link` | 320 tia, ~200 tia hợp lệ, 0.55–5.92m, lidar quay 8.5 Hz |
| `/odom` | `odom`→`base_footprint` | x/y/yaw/v/w đúng định dạng |
| `/imu/data` | `imu_link` | gyro ≈ 0 khi đứng yên, \|accel\| = 10.08 m/s² (trọng lực) |
| `/tf` | `odom`→`base_footprint` | đúng |

**Vấn đề đang tồn tại:** cả 4 topic chỉ phát ~**1 Hz** thay vì 50 Hz
(`/odom`, `/imu/data`, `/tf`) và 8.5 Hz (`/scan`). Dấu thời gian trong message
cách nhau ~967 ms nên đây là phía ESP32 chậm, không phải công cụ đo. Nav2 cần
odometry ≥ 20 Hz nên **phải sửa trước khi chạy B5**. Xem mục kế tiếp.

**Chưa kiểm chứng:** giao diện Gazebo/RViz (tôi chạy không màn hình), toàn bộ
quy trình SLAM trên xe thật, và việc xe di chuyển theo `/cmd_vel`.
