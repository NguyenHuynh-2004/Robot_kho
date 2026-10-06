# Hướng Dẫn Khởi Chạy Dự Án AGV (robot_ws_3)

Xe tự hành trên **sa bàn 5×5** (ô 60 cm). Bạn nhập hàng, cột và hướng đích; xe tự
tìm đường **chỉ đi trên các cạnh**, xoay tại chỗ ở chỗ rẽ và tự né vật cản.

Mở các terminal **lần lượt** — terminal trước khởi động xong mới mở terminal sau.

| Kịch bản | Dùng khi | Số terminal |
|---|---|---|
| 1️⃣ Vẽ và lưu bản đồ bằng tay cầm | phòng mới / đồ đạc thay đổi | 5 + 1 PowerShell |
| 2️⃣ Chạy một xe thật trên sa bàn | **dùng hằng ngày** | 4 |
| 3️⃣ Chạy hai xe (mô phỏng) | thử điều phối hai xe | 2 |

> [!IMPORTANT]
> **Mọi terminal** đều bắt đầu bằng hai dòng này (đã ghi sẵn trong từng khung lệnh
> để bạn chép cho tiện):
> ```bash
> cd ~/robot_ws_3
> source install/setup.bash
> ```

> [!NOTE]
> **Máy mới / lần đầu:** làm [Chuẩn bị máy mới](SA_BAN_VA_TAI_LIEU.md#h1-chuẩn-bị-máy-mới-chỉ-làm-một-lần-cho-mỗi-máy)
> và [Nạp firmware](SA_BAN_VA_TAI_LIEU.md#h3-nạp-firmware-xe-thật) trước. Sa bàn,
> bảng tra sự cố, nối dây, thuật toán: xem [SA_BAN_VA_TAI_LIEU.md](SA_BAN_VA_TAI_LIEU.md).

> [!IMPORTANT]
> **Khởi động lại xe = tắt hẳn nguồn vài giây rồi bật**, để xe đứng yên 2 giây đầu
> (BNO055 đo độ lệch gyro). Chỉ bấm nút reset ESP32 có lúc làm BNO055 trả toàn số 0.

---

## 1️⃣ Kịch Bản 1: Vẽ Và Lưu Bản Đồ Bằng Tay Cầm (Cần 5 Terminals + 1 PowerShell)

Làm trước lần chạy sa bàn đầu tiên, hoặc khi đồ đạc trong phòng thay đổi nhiều.

> [!IMPORTANT]
> **Gốc bản đồ = chỗ xe đứng lúc bật SLAM.** Đặt xe ở chỗ dễ nhớ và nhớ hướng đầu xe.

### Terminal 1: Cầu Nối ESP32
```bash
cd ~/robot_ws_3
source install/setup.bash
ros2 launch my_bot_bringup bringup_real.launch.py
```
Bật nguồn xe. Log agent phải hiện `session established`.

### Terminal 2: SLAM
```bash
cd ~/robot_ws_3
source install/setup.bash
ros2 launch my_bot_bringup slam.launch.py use_sim_time:=false
```

### PowerShell Trên Windows: Đọc Tay Cầm
Tay cầm USB không hiện trong WSL, nên đọc bên Windows rồi đẩy qua UDP. Mở
**PowerShell** (không phải WSL) và giữ cửa sổ này mở:
```
python \\wsl.localhost\Ubuntu\home\huynh\robot_ws_3\tools\windows_gamepad_bridge.py
```
Nó in `re=... tien=... LB=... RB=...` mỗi 2 giây.

### Terminal 3: Nhận Tay Cầm → `/joy`
```bash
cd ~/robot_ws_3
source install/setup.bash
python3 tools/joy_udp_bridge.py
```

### Terminal 4: `/joy` → Lệnh Chạy Xe
```bash
cd ~/robot_ws_3
source install/setup.bash
ros2 launch my_bot_bringup joy_teleop.launch.py start_joy_node:=false
```

> [!TIP]
> **Điều khiển:** **giữ LB rồi đẩy cần trái**; thả LB là xe dừng. Giữ thêm **RB** để
> chạy nhanh gấp đôi. `start_joy_node:=false` là bắt buộc trên WSL.

### Terminal 5: RViz Xem Bản Đồ
```bash
cd ~/robot_ws_3
source install/setup.bash
rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz
```
Lái chậm đi khắp phòng, quay tại chỗ vài vòng ở các góc, tới khi tường đen trong
RViz kín hết.

### Lưu Bản Đồ (SLAM ở Terminal 2 vẫn phải đang chạy)
Mở thêm một terminal:
```bash
cd ~/robot_ws_3
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f src/my_bot_bringup/config/real_room_map
colcon build --symlink-install
```
Ra hai file `real_room_map.pgm` và `real_room_map.yaml`. `colcon build` là bắt buộc
để launch thấy bản đồ mới.

> [!WARNING]
> - Đừng lưu đè `my_room_map` — đó là bản đồ của mô phỏng Gazebo.
> - Vẽ xong bản đồ mới phải **căn lại sa bàn** cho khớp vạch gạch:
>   [Căn sa bàn](SA_BAN_VA_TAI_LIEU.md#căn-sa-bàn-cho-khớp-vạch-gạch-sau-khi-vẽ-bản-đồ-mới).
> - Tắt hết các terminal của kịch bản này (nhất là **SLAM**) trước khi sang Kịch bản 2.

---

## 2️⃣ Kịch Bản 2: Chạy Một Xe Thật Trên Sa Bàn (Cần 4 Terminals)

Kịch bản dùng hằng ngày.

### Terminal 1: Cầu Nối ESP32
```bash
cd ~/robot_ws_3
source install/setup.bash
ros2 launch my_bot_bringup bringup_real.launch.py
```
Bật nguồn xe, chờ log agent có `session established`.

### Terminal 2: Nav2 (gồm sẵn bản đồ + định vị AMCL)
```bash
cd ~/robot_ws_3
source install/setup.bash
ros2 launch my_bot_bringup navigation.launch.py use_sim_time:=false
```
Chờ log có `Managed nodes are active`, rồi **đặt vị trí ban đầu** (xe đứng yên ở đâu
cũng được):
```bash
cd ~/robot_ws_3
source install/setup.bash
python3 tools/auto_initial_pose.py
```

> [!WARNING]
> **Đừng quên `use_sim_time:=false`** — thiếu nó node chờ `/clock` mãi mà không báo
> lỗi gì. SLAM của Kịch bản 1 phải đã tắt.

### Terminal 3: RViz
```bash
cd ~/robot_ws_3
source install/setup.bash
rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz
```
Fixed Frame = `map`. Thêm **Add → By topic → `/grid_visualization` → MarkerArray**
để thấy sa bàn. **Điểm quét đỏ phải trùng tường đen** trước khi cho xe chạy — lệch thì
bấm **2D Pose Estimate**, bấm vào chỗ xe thật rồi kéo theo hướng đầu xe.

### Terminal 4: Nhập Đích
```bash
cd ~/robot_ws_3
source install/setup.bash
python3 grid_navigator.py --real
```
Dòng đầu phải in `Che do: XE THAT` và `Đã đối chiếu sa bàn với bản đồ: 24/25 điểm`.

Nhập lần lượt:

| Hỏi | Nhập | Ví dụ |
|---|---|---|
| Hàng | `1`–`5` (1 = dưới cùng) | `3` |
| Cột | `1`–`5` (1 = bên trái) | `5` |
| Hướng tại đích | `D` Đông (phải) · `T` Tây (trái) · `B` Bắc (lên) · `N` Nam (xuống) · Enter = giữ hướng | `d` |

Xe tới đích (sai số ≤ 3 cm) rồi xoay về hướng đã chọn (≤ 3°). Gõ `q` để thoát.

> [!TIP]
> **Dòng đầu Terminal 4** luôn hiện, kể cả lúc chờ nhập: góc IMU, vị trí, hướng xe
> (≈ Đông/Tây/Nam/Bắc) và điểm lưới gần nhất. Không muốn: thêm `--no-status`.
>
> Lần đầu hãy chọn điểm gần (vd hàng 3 cột 3 → hàng 3 cột 4, 60 cm).

> [!CAUTION]
> **Dừng khẩn cấp:** Ctrl-C Terminal 2 (Nav2 ngừng phát lệnh, ESP32 tự dừng động cơ
> sau 0.5 s) hoặc tắt công tắc nguồn. Đừng bấm LB tay cầm khi Nav2 đang lái — hai
> lệnh xen nhau làm xe giật.

---

## 3️⃣ Kịch Bản 3: Chạy Hai Xe — Mô Phỏng (Cần 2 Terminals)

Thử bộ điều phối hai xe: giữ chỗ, ưu tiên **xe gần đích hơn**, tự gỡ kẹt. Chạy trên
`ROS_DOMAIN_ID=87` nên **không đụng tới xe thật**. Hai xe ảo nhưng Nav2 là Nav2 thật.

### Terminal 1: Bật Mô Phỏng (bản đồ, 2 xe ảo, 2 Nav2)
```bash
cd ~/robot_ws_3
source install/setup.bash
tools/fleet_sim.sh up
```
Chờ khoảng 40 giây tới khi in `Nav2 agv1 ... san sang` và `Nav2 agv2 ... san sang`.
Vị trí đầu mặc định: xe 1 ở (hàng 3, cột 1), xe 2 ở (hàng 3, cột 5). Muốn khác:
```bash
AGV1="3 1 0" AGV2="3 5 180" tools/fleet_sim.sh up     # hàng cột hướng_độ
```

### Terminal 2: Bộ Điều Phối
```bash
cd ~/robot_ws_3
source install/setup.bash
ROS_DOMAIN_ID=87 python3 traffic_manager.py agv1 agv2
```

| Nhập | Nghĩa |
|---|---|
| `1 5 5 d` | xe 1 tới hàng 5 cột 5, quay Đông |
| `2 1 1` | xe 2 tới hàng 1 cột 1, giữ hướng |
| `s` | dừng tất cả |
| `q` | thoát |

### Tắt Mô Phỏng
```bash
cd ~/robot_ws_3
tools/fleet_sim.sh down
```

> [!TIP]
> - **Thả vật cản ảo** ở (x, y) mét:
>   `ROS_DOMAIN_ID=87 ros2 topic pub --once /sim/obstacles std_msgs/msg/String "{data: 'x,y'}"`
>   (gửi `''` để bỏ).
> - **RViz** (cũng `ROS_DOMAIN_ID=87`): topic `/fleet/markers` — đường còn lại và ô
>   đang giữ của từng xe.

Quy tắc điều phối và kết quả kiểm thử:
[SA_BAN_VA_TAI_LIEU.md — Phụ lục F](SA_BAN_VA_TAI_LIEU.md#f-quy-tắc-điều-phối-hai-xe).
