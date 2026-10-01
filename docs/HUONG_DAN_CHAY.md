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

### Bấm nút BOOT khi nạp

Bo CH340 này **không tự vào chế độ nạp**. Khi Arduino IDE hiện
`Connecting....`, **giữ nút BOOT** cho tới khi thanh tiến trình chạy. Không giữ
thì báo:

```
A fatal error occurred: Failed to connect to ESP32:
Wrong boot mode detected (0x13)! The chip needs to be in download mode.
```

### Nạp xong phải thấy 5 dòng này

Mở **Serial Monitor 115200**, nhấn nút EN để khởi động lại, rồi soi:

```
>>> Reset: CAP NGUON                                  ← nếu là SUT AP thì lỗi nguồn
Wi-Fi OK, IP xe: 192.168.1.150
Da thay agent!                                        ← chưa có nghĩa là agent chưa chạy
Dong ho: tick1 549 ms, sync -> 0, tick2 990 ms        ← sync phải = 0
LiDAR start(): OK
```

Riêng 4 dòng lỗi này là **bình thường, bỏ qua**:

```
E (318) gpio: gpio_pulldown_en(123): GPIO number error (input-only pad has no internal PD)
E (324) gpio: gpio_pullup_en(85):   GPIO number error (input-only pad has no internal PU)
```

Thư viện ESP32Encoder cố bật điện trở kéo trên GPIO 34/35, mà hai chân đó chỉ
vào được nên không có điện trở nội. Vô hại — nhưng đúng như cảnh báo ở mục
Encoder bên dưới: **nếu encoder của bạn là loại hở cực thu thì kênh B bắt buộc
phải có điện trở 10kΩ kéo lên ngoài**, không thì odometry loạn.

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
| 5 | Đọc dòng thứ hai (dòng chẩn đoán) | `pub/s: scan 8 odom 50 imu 50` |

Lệnh cho bước 2 (xe **sẽ chạy** — kê bánh lên khỏi mặt đất trước):

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

Cả 5 bước đúng rồi mới hiệu chỉnh `WHEEL_RADIUS_M` / `WHEEL_SEPARATION_M`.

### Đọc dòng chẩn đoán

Mỗi giây firmware in **hai** dòng. Dòng thứ hai là để soi hiệu năng:

```
ros_task 605 vong/s | pub/s: scan 9 odom 50 imu 50 | max us: spin 1145, imu 6983,
                              odom 3592, scan 1752, sync 6, ca_vong 10061
```

| Cột | Bình thường | Nghĩa khi lệch |
|---|---|---|
| `ros_task …/s` | 500–700 | thấp → vòng lặp bị chặn, soi cột `max us` nào lớn nhất |
| `pub/s scan` | 8–9 | đây là nhịp quay LiDAR, không phải 50 |
| `pub/s odom` / `imu` | 49–50 | thấp hơn → xem `ca_vong` |
| `spin` | < 2000 | ~980000 → lỗi `spin_some` chặn, xem Bảng tra sự cố |
| `imu` | 5000–9000 | lớn hơn → dây I2C dài/nhiễu |
| `sync` | < 300 | ~600000 là `ping_agent` chặn — **chỉ xảy ra khi xe đứng yên**, xem bên dưới |
| `ca_vong` | 8000–12000 | 750000 → có lệnh chặn trong vòng lặp |
| `loi pub` (dòng 1) | 0 | > 0 → message vượt MTU hoặc hết bộ đệm |

> **Về cột `sync`:** `rmw_uros_ping_agent` không tôn trọng timeout, đo được nó
> chặn tới 1050 ms. Nhưng khi có `/cmd_vel` về thì điều đó tự chứng minh agent
> còn sống, nên firmware **tắt ping** trong lúc đang được điều khiển. Vì vậy
> đỉnh `sync ~600000` lúc xe đứng yên là **bình thường**; nếu nó vẫn xuất hiện
> trong lúc Nav2 hoặc tay cầm đang lái thì mới là bất thường.

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
cd ~/robot_ws_3
python3 tools/check_qos.py          # QoS có khớp không
python3 tools/check_tf.py           # cây TF có frame "odom" không
python3 tools/check_esp32_rates.py  # nhịp và độ trễ
```

`check_tf.py` **phải** in `odom -> lidar_link  OK`. Không có frame `odom` thì
mọi thứ phía sau vô nghĩa: slam_toolbox sẽ loại sạch scan và không bao giờ
sinh ra bản đồ, còn RViz báo `Frame [map] does not exist`.

Mong đợi: `/scan` ~8.7 Hz, `/odom` `/imu/data` `/tf` ~50 Hz, và `lech stamp`
trung vị trong khoảng **−150 → +150 ms**.

> **Đừng đo bằng node tự viết subscribe cả 4 topic một lúc.** Subscriber Python
> đọc không kịp 200 msg/s, message dồn hàng đợi, và nó báo sai *cả* nhịp *lẫn*
> độ trễ — tôi đã bị chính lỗi này: đo ra 14 Hz / trễ 798 ms, trong khi thực tế
> là 50 Hz / trễ −40 ms. Dấu hiệu nhận ra: khoảng giữa hai lần *nhận* ngắn hơn
> chu kỳ phát (10.7 ms so với 20 ms) — tức đang đọc từ đống tồn.

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
| `ros2 topic echo` báo không có publisher dù topic tồn tại | lỗi discovery của **CLI** `ros2` trong WSL | dùng `tools/check_esp32_rates.py` |
| `Connecting....` rồi `Wrong boot mode detected (0x13)` | bo CH340 không tự vào chế độ nạp | **giữ nút BOOT** lúc đang `Connecting....` |
| Mọi topic chỉ ~1 Hz, cột `spin` ~980000 | `rclc_executor_spin_some` **chặn tới khi có dữ liệu đến**, không tôn trọng timeout → mọi thứ publish sau nó tụt xuống nhịp của `/cmd_vel` | firmware đã truyền timeout `0`; nếu tái diễn thì kiểm dòng `rclc_executor_set_timeout -> 0 (OK)` |
| Nav2 báo `message too old` / `extrapolation`, stamp cách nhau đúng 1000 ms | đồng hồ trong `rmw_uros_epoch_nanos()` chỉ có độ phân giải **1 giây** | đã sửa: firmware neo epoch rồi tự chạy bằng `esp_timer`. Kiểm dòng boot `Dong ho: tick1 … sync -> 0, tick2 …` |
| Stamp đúng nhịp nhưng cả cụm trễ ~1 giây | `sync_session` tính offset từ chính đồng hồ 1 giây đó nên **offset sai tới 1 giây** | đã sửa: chờ đúng cạnh tick rồi mới `sync_session` |
| `E (318) gpio: gpio_pulldown_en(123)` lúc boot | ESP32Encoder bật điện trở kéo trên GPIO 34/35 — hai chân chỉ-vào, không có điện trở nội | bỏ qua. Nhưng encoder hở cực thu thì **phải** có 10kΩ kéo lên ngoài |
| LiDAR quét tới xa nhưng vùng trắng không lan ra, phần lớn bản đồ vẫn **xám (unknown)** | ô chỉ được tô khi `cellPassCnt > min_pass_through` (Karto `UpdateCell`), không thoả thì giữ nguyên xám. Mật độ tia loãng dần theo khoảng cách: 320 tia/360° → xa hơn **2.55 m** là hai tia liền nhau nhảy cóc qua ô 5 cm ở giữa, nên mỗi vòng quét chỉ cho ô ≤ 1 lượt | `min_pass_through` 2 (không nâng lên 3), và hạ `minimum_travel_distance`/`heading` xuống 0.2 để nhiều vị trí quan sát hơn. Cả hai đã chỉnh trong `config/slam_async.yaml` |
| Bản đồ ra từng mảng trắng rời rạc, rìa tua hình nan quạt, **gần như không có tường đen** | điểm trúng tường bị rải ra quá nhiều ô nên không ô nào đủ tỉ lệ trúng/đi-qua → vấn đề ở **ước lượng vị trí**, không phải ở ngưỡng. Nghi phạm số một là `LIDAR_ANGLE_FLIP` sai (ảnh quét lật gương) | `python3 tools/check_lidar_angle.py`, cho xe quay tại chỗ. Lưu ý: sai `LIDAR_ANGLE_OFFSET_DEG` **không** gây ra cảnh này — lệch góc cố định chỉ làm bản đồ bị xoay chứ vẫn sắc nét |
| Đẩy cần lên xe chạy lùi, đẩy trái xe rẽ phải | đảo **cả hai** động cơ cho đúng triệu chứng này: `v>0` thành lùi, `w>0` thành rẽ phải | chạy `python3 tools/check_direction.py` để biết lỗi ở tay cầm hay ở động cơ, rồi làm theo kết luận nó in ra. Nếu là động cơ thì phải đảo **cả bốn** cờ `INVERT_MOTOR_*` và `INVERT_ENC_*`, chỉ đảo động cơ sẽ bị ESTOP |
| Bản đồ đầy chấm đen nhỏ rải rác khắp vùng trống | LiDAR có ~6% tia rác (đo bằng `tools/check_scan_noise.py`), **và** `occupancy_threshold` mặc định `0.1` khuếch đại nó: chỉ cần 1 tia trúng trên 9 lần đi qua sạch là ô đó thành vật cản vĩnh viễn | đã sửa trong `config/slam_async.yaml`: `occupancy_threshold` 0.3, `min_pass_through` 3, `min_laser_range` 0.12, `max_laser_range` 8.0 |
| RViz không hiện hình xe | display **RobotModel bị tắt** trong `nav2_default_view.rviz` (ô tick trống, chữ đen thay vì xanh, mục con bị xám) | tích vào ô đó. Muốn giữ thì **File → Save Config As** vào `src/my_bot_bringup/rviz/` — cấu hình gốc nằm trong `/opt/ros/jazzy/` nên không lưu được |
| RViz: `Frame [map] does not exist` + `No map received`, log đầy `Message Filter dropping message: frame 'lidar_link' … queue is full` | **lệch QoS trên `/tf`**: firmware phát best-effort còn `tf2_ros::TransformListener` luôn subscribe reliable → tf2 không nhận gì → frame `odom` không tồn tại → slam_toolbox loại sạch scan | đã sửa: `/tf` và `/odom` đổi sang `rclc_publisher_init_default` (reliable). Kiểm bằng `python3 tools/check_qos.py` |
| Cột `sync` có đỉnh ~600000 | `ping_agent` chặn | bình thường **khi xe đứng yên**; firmware tắt ping khi có `/cmd_vel` |

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
| `/scan` | `lidar_link` | 320 tia, ~230 tia hợp lệ, 0.18–7.89m, lidar quay 8.7 Hz |
| `/odom` | `odom`→`base_footprint` | x/y/yaw/v/w đúng định dạng |
| `/imu/data` | `imu_link` | gyro ≈ 0 khi đứng yên, \|accel\| = 10.12 m/s² (trọng lực) |
| `/tf` | `odom`→`base_footprint` | đúng |

### Ba lỗi đã tìm ra và sửa trên phần cứng thật

| | trước | sau |
|---|---|---|
| nhịp phát `/odom` `/imu` `/tf` | 1 Hz | **50 Hz** |
| nhịp phát `/scan` | 1 Hz | **8.7 Hz** |
| độ phân giải stamp | 1000 ms | **20 ms** |
| stamp lệch đồng hồ máy | +1158 ms | **−40 ms** (trung vị) |
| `ca_vong` lâu nhất | 751 000 µs | 8 000–12 000 µs |

1. **`rclc_executor_spin_some` chặn tới khi có dữ liệu đến**, không tôn trọng
   timeout. Đo được `spin` = 965–990 ms trong một vòng ~1000 ms. Mọi thứ publish
   nằm sau nó nên toàn bộ topic tụt xuống nhịp của `/cmd_vel`. Truyền timeout
   `0` thì `spin` xuống 500–2000 µs.
2. **`rmw_uros_epoch_nanos()` chỉ có độ phân giải 1 giây.** Chứng minh bằng cách
   in stamp thô: trường `nanosec` đứng yên hàng chục giây, chỉ `sec` tăng, và
   chỉ đổi đúng lúc `sync_session()` chạy. 50 message trong một giây mang y hệt
   một mốc thời gian. Sửa: neo epoch một lần rồi tự chạy bằng `esp_timer`.
3. **Chính cái offset của `sync_session` cũng sai tới 1 giây**, vì nó tính
   `((t2-t1)+(t3-t4))/2` với `t1`/`t4` đọc từ đúng đồng hồ 1 giây đó. Sửa: chờ
   đúng khoảnh khắc đồng hồ thô vừa nhảy (lúc đó nó đúng) rồi mới `sync_session`.

### Chưa kiểm chứng

- **Xe di chuyển theo `/cmd_vel`** — chưa bao giờ thử, xe vẫn đang đứng yên.
- **Việc tắt ping khi đang được điều khiển.** Firmware chỉ chạy
  `ping_agent`/`sync_session` khi mất `/cmd_vel` quá hạn. Lúc xe đứng yên thì
  điều kiện đó luôn đúng nên vẫn thấy `sync ~600000` mỗi 5 giây — đúng như
  thiết kế. Nhưng **chưa có ai xác nhận nó im lúc đang lái**: cần chạy B4 hoặc
  B5 rồi soi cột `sync`.
- Vẫn còn **14 khoảng trống > 200 ms trong 20 giây** (lớn nhất 866 ms) lúc xe
  đứng yên. Khoảng 4 cái là các lần ping nói trên; số còn lại chưa quy được cho
  ai — có thể vẫn là công cụ đo hoặc WSL.
- Giao diện Gazebo/RViz (chạy không màn hình) và toàn bộ quy trình SLAM trên xe
  thật.
