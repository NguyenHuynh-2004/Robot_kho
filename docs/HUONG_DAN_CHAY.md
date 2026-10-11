# Hướng Dẫn Khởi Chạy Dự Án AGV (robot_ws_3)

Xe tự hành trên **sa bàn 5×5** (ô 60 cm). Bạn nhập hàng, cột và hướng đích; xe tự
tìm đường **chỉ đi trên các cạnh**, xoay tại chỗ ở chỗ rẽ và tự né vật cản.

Mở các terminal **lần lượt** — terminal trước khởi động xong mới mở terminal sau.

| Kịch bản | Dùng khi | Số terminal |
|---|---|---|
| 1️⃣ Vẽ và lưu bản đồ bằng tay cầm | phòng mới / đồ đạc thay đổi | 5 + 1 PowerShell |
| 2️⃣ Chạy một xe thật trên sa bàn | **dùng hằng ngày** | 4 |
| 3️⃣ Chạy hai xe (mô phỏng) | thử điều phối hai xe | 4 |
| 4️⃣ Chạy hai xe thật | **hai xe cùng chạy** trên sa bàn thật | 3 |

> [!IMPORTANT]
> **Mỗi xe một file firmware riêng**: xe 1 nạp `Xe_1\Xe_1.ino`, xe 2 nạp `Xe_2\Xe_2.ino`
> ([Nạp firmware](SA_BAN_VA_TAI_LIEU.md#h3-nạp-firmware-xe-thật)). Mọi topic của xe
> nằm trong `/agv1/...` hoặc `/agv2/...`, nên chạy một xe phải ghi rõ **`xe:=1`** hoặc
> **`xe:=2`** ở lệnh cầu nối ESP32.

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
ros2 launch my_bot_bringup bringup_real.launch.py xe:=2     # số xe đang dùng: 1 hoặc 2
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
ros2 launch my_bot_bringup bringup_real.launch.py xe:=2     # số xe đang dùng: 1 hoặc 2
```
Bật nguồn xe, chờ log agent có `session established` và dòng
`Chuyen tiep /agv2/{scan, imu/data, joint_states} -> goc` (topic của xe được chuyển
về `/scan`, `/cmd_vel`... như cũ, các terminal sau giữ nguyên).

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

## 3️⃣ Kịch Bản 3: Chạy Hai Xe — Mô Phỏng (Cần 4 Terminals)

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

Terminal hỏi lần lượt **xe 1 rồi xe 2**, giống Kịch bản 2:

| Hỏi | Nhập | Ví dụ |
|---|---|---|
| Xe 1 - Hàng đích | `1`–`5` (1 = dưới cùng) | `3` |
| Xe 1 - Cột đích | `1`–`5` (1 = bên trái) | `5` |
| Hướng tại đích | `D` Đông · `T` Tây · `B` Bắc · `N` Nam · Enter = giữ hướng | `d` |
| Xe 2 - Hàng / Cột / Hướng | như trên | `3`, `1`, `t` |

Phải **nhập đủ cả hai xe** thì hai xe mới **cùng xuất phát**. Nhập sai (ngoài sa bàn,
ô sát vật cản, trùng đích với xe kia) thì báo lỗi và hỏi lại xe đó. Khi cả hai xe tới
nơi, terminal in `Xe 1 ĐÃ TỚI ...`, `Xe 2 ĐÃ TỚI ...` rồi hỏi lượt tiếp. Gõ `q` để thoát,
Ctrl-C để dừng khẩn.

> [!NOTE]
> Đích của xe này được là ô xe kia **đang đỗ** (vd hai xe đổi chỗ): xe đang đỗ sẽ tự
> né sang ô bên cạnh.

### Terminal 3: RViz
```bash
cd ~/robot_ws_3
source install/setup.bash
ROS_DOMAIN_ID=87 rviz2 -d tools/fleet.rviz
```
Thấy bản đồ, lưới sa bàn (điểm đỏ = bị chặn), **hai xe** (xe 1 xanh dương, xe 2 cam,
mũi tên đen = hướng đầu xe), **đường còn lại** và **ô đang giữ** của từng xe. Xe mất
liên lạc hiện mờ kèm chữ `MAT LIEN LAC`.

> [!NOTE]
> Không dùng file RViz của Kịch bản 2 ở đây: mỗi xe có cây TF riêng (`/agv1/tf`,
> `/agv2/tf`). `tools/fleet_sim.sh` tự chạy `tools/fleet_viz.py` gộp hai cây (frame
> `agv1/...`, `agv2/...`) nên `tools/fleet.rviz` thấy được ảnh quét, costmap, đường đi của
> từng xe. Xe ảo không có URDF nên mục "Mô hình xe" và "Hạt AMCL" để trống — bình thường.

### Terminal 4: Gazebo (xem 3D + đặt vật cản)
```bash
cd ~/robot_ws_3
source install/setup.bash
ROS_DOMAIN_ID=87 python3 tools/fleet_gazebo.py
```
Cửa sổ Gazebo hiện **tường phòng** (dựng từ bản đồ), **vạch sa bàn** (nút đỏ), **hai xe**
(xe 1 xanh dương, xe 2 cam, mũi trắng = đầu xe) và **6 trụ đỏ** (vật cản) xếp hàng chờ
dưới sa bàn. Xe trong Gazebo chạy theo hai xe ảo của Terminal 1. Ctrl-C để tắt.

| Muốn | Làm |
|---|---|
| Dời khung nhìn | giữ chuột trái, kéo |
| Xoay khung nhìn | giữ con lăn (hoặc Shift + chuột trái), kéo |
| Phóng to / thu nhỏ | lăn con lăn |

**Đặt vật cản bằng chuột:**

1. Bấm phím **`T`** (hoặc biểu tượng mũi tên chữ thập ở góc trên bên trái) → chế độ dời vật.
2. Bấm vào **một trụ đỏ** → hiện 3 mũi tên.
3. Kéo **mũi tên đỏ** (dọc trục x, sang phải/trái) và **mũi tên xanh lá** (trục y, lên/xuống)
   để đưa trụ vào sa bàn. **Đừng kéo mũi tên xanh dương** (lên cao).
4. Thả chuột. Terminal 4 in vị trí, ví dụ
   `Vat can tren sa ban (1): (-0.55, +0.34) gan nut hang 4 cot 3 (30 cm)`, và LiDAR hai xe
   ảo thấy trụ ngay. Xe đang chạy gặp trụ thì tự đổi đường như vật cản thật.
5. Bấm **`Esc`** để thôi chế độ dời vật. **Bỏ vật cản** = kéo trụ ra khỏi sa bàn (về hàng chờ).

> [!NOTE]
> - Gazebo không tính vật lý, không lái xe: **xe** vẫn do Terminal 1 điều khiển, Gazebo
>   chỉ vẽ lại. Còn **vật cản** thì lấy từ Gazebo: trụ nào nằm trong sa bàn (tính
>   rộng thêm 25 cm ra ngoài nút biên) là vật cản, trụ ở ngoài thì không.
> - Mở Gazebo sẽ **xóa vật cản cũ** đặt bằng lệnh trước đó. Lệnh `ros2 topic pub ...
>   /sim/obstacles` (ở dưới) vẫn dùng được: trụ đỏ tự nhảy tới chỗ đã gửi.
> - Tắt Terminal 4 thì mô phỏng vẫn chạy bình thường, vật cản đang có vẫn giữ.

### Tắt Mô Phỏng
```bash
cd ~/robot_ws_3
tools/fleet_sim.sh down          # tắt cả Gazebo của Terminal 4
```

> [!TIP]
> - **Thả vật cản ảo bằng lệnh** (không cần Gazebo) ở (x, y) mét, nhiều vật cách nhau `;`:
>   `ROS_DOMAIN_ID=87 ros2 topic pub --once /sim/obstacles std_msgs/msg/String "{data: 'x,y'}"`
>   (gửi `''` để bỏ). Ví dụ giữa nút hàng 3 cột 3 và hàng 4 cột 3: `'-0.545,0.335'`.

Quy tắc điều phối và kết quả kiểm thử:
[SA_BAN_VA_TAI_LIEU.md — Phụ lục F](SA_BAN_VA_TAI_LIEU.md#f-quy-tắc-điều-phối-hai-xe).

### Thêm: Đo Hàng Bằng Camera (mô phỏng, rộng × cao)

Xe tới ô (5,c) quay Bắc → **LiDAR** khớp mặt trước hàng (khoảng cách Z, góc lệch) →
**camera nhìn thẳng** chụp, tách hàng khỏi nền, tìm 4 mép → chiếu từng mép lên mặt
phẳng mặt hàng → **rộng × cao** → bảng phân loại → xe tự chạy về ô (1,k). Xe không chở
hàng. Cấu hình (camera, khoảng đặt hàng, bảng loại): `src/my_bot_bringup/config/cargo.yaml`.

Bật Terminal 1 + 2 như trên (Terminal 1 thêm `FAKE_LIDAR_NOISE=0.01 FAKE_RAYS=320` cho
LiDAR ảo nhiễu giống X2), rồi:

```bash
# Terminal 5: hàng hoá + camera ảo — "cột:rộng:cao:sâu[:xoay_độ[:lệch_ngang]]" (m)
ROS_DOMAIN_ID=87 python3 tools/sim_cargo.py agv1 agv2 --goods "2:0.15:0.20:0.12, 4:0.10:0.12:0.10:5"
# Terminal 6: nhiệm vụ đo hàng — nhập "1 2" = xe 1 đo hàng ở (5,2); "2 4" = xe 2 ở (5,4)
ROS_DOMAIN_ID=87 python3 cargo_task.py agv1 agv2
```
Đổi hàng lúc đang chạy:
`ROS_DOMAIN_ID=87 ros2 topic pub --once /sim/cargo_set std_msgs/msg/String "{data: '3:0.2:0.18:0.15'}"`.
RViz (Terminal 3) thấy hộp hàng màu nâu ở phía Bắc hàng 5 và chữ kết quả trên ô hàng 1.
Kết quả + ảnh có vẽ khung: `~/agv_logs/cargo/` (`results.csv`).

| Đo trong mô phỏng (2026-10-12) | Kết quả |
|---|---|
| 9 lần đo, hàng 9–30 cm, xoay tới 21°, xe lệch 3 cm / 3° | sai **≤ 3,5 mm** |
| Hai xe cùng loại hàng → cùng ô (1,1) | xe giao trước lùi lên hàng 2 nhường ô |
| Hàng rộng 40 cm (quá khung hình) / ô không có hàng | báo **không đo được**, xe không giao |

> [!WARNING]
> Giới hạn (camera cao 11 cm, hàng cách tâm ô 35 cm): rộng ≤ ~33 cm, cao 11–25 cm. Hàng
> thấp hơn camera thì thấy cả mặt trên → chiều cao sai vài mm (có cảnh báo). Kích thước
> sát ngưỡng bảng loại (vd rộng 12,0 cm với ngưỡng 12 cm) có thể rơi sang loại bên cạnh.
> Xe thật cần thêm ESP32-CAM + hiệu chuẩn camera — chưa làm.

---

## 4️⃣ Kịch Bản 4: Chạy Hai Xe Thật (Cần 3 Terminals)

Hai xe thật cùng chạy trên sa bàn theo đúng luật của mô phỏng: giữ chỗ trước, ưu tiên
**xe gần đích hơn**, xe đang đỗ ở đích của xe kia tự né, tự gỡ kẹt. Mỗi xe có định vị AMCL
và Nav2 riêng, dùng chung bản đồ và một micro-ROS agent.

> [!IMPORTANT]
> **Chuẩn bị:**
> - **Firmware:** xe 1 nạp `Xe_1\Xe_1.ino`, xe 2 nạp `Xe_2\Xe_2.ino` (đã nạp ngày
>   2026-10-10; chỉ nạp lại khi firmware thay đổi —
>   [cách nạp](SA_BAN_VA_TAI_LIEU.md#h3-nạp-firmware-xe-thật)).
> - **Pin đầy**, công tắc nguồn động cơ bật.
> - **Tắt hết** các terminal của Kịch bản 1 và 2 (cầu nối ESP32, Nav2, `grid_navigator`):
>   chạy song song thì hai bản đồ, hai agent đè nhau.
> - **Đặt xe:** xe 1 ở **(hàng 3, cột 1) mũi hướng Đông**, xe 2 ở **(hàng 3, cột 5) mũi
>   hướng Tây**. Tâm xe trên nút, mũi xe dọc vạch.

### Terminal 1: Bật Hệ Hai Xe
Bật nguồn **cả hai xe**, để yên 2 giây, rồi:
```bash
cd ~/robot_ws_3
source install/setup.bash
tools/fleet_real.sh up
```
Chờ tới khi in đủ (30 giây – 2 phút; Nav2 một xe có lúc không lên lần đầu, script tự bật lại):
```
Cho xe agv1 (/agv1/scan) co
Cho xe agv2 (/agv2/scan) co
Nav2 agv1 (lan 1)...... san sang
Nav2 agv2 (lan 1)...... san sang
Da bat. ...
```
Báo `KHONG THAY` → xe đó chưa bật nguồn hoặc nạp nhầm file.

**Định vị chính xác** (cùng terminal, sau khi `Da bat`):
```bash
python3 tools/fleet_localize.py
```
Mỗi xe phải in `da gui cho AMCL`, độ dời thường vài cm. Xe không đứng ở chỗ đã khai báo
(`AGV1`/`AGV2`) thì công cụ **tự dò cả sa bàn** (25 nút × 4 hướng, vài giây) và in
`xe dang o (hàng,cột) huong ...` — kiểm tra đúng chỗ bạn đặt xe. Vẫn báo `KHONG GUI` → xe
không đứng trên nút nào, hoặc quanh xe có nhiều đồ đạc không có trên bản đồ.

Đặt xe ở chỗ khác thì khai báo khi bật:
```bash
AGV1="3 1 0" AGV2="3 5 180" tools/fleet_real.sh up     # hàng cột hướng_độ
```

### Terminal 2: Bộ Điều Phối
```bash
cd ~/robot_ws_3
source install/setup.bash
python3 traffic_manager.py agv1 agv2
```
Hỏi lần lượt **xe 1 rồi xe 2**:

| Hỏi | Nhập | Ví dụ |
|---|---|---|
| Xe 1 - Hàng đích | `1`–`5` (1 = dưới cùng) | `3` |
| Xe 1 - Cột đích | `1`–`5` (1 = bên trái) | `5` |
| Hướng tại đích | `D` Đông · `T` Tây · `B` Bắc · `N` Nam · Enter = giữ hướng | `t` |
| Xe 2 - Hàng / Cột / Hướng | như trên | `3`, `1`, `d` |

Phải **nhập đủ cả hai xe** thì hai xe mới **cùng xuất phát**. Muốn một xe **đứng yên**:
nhập đúng ô nó đang đứng và Enter ở hướng. Khi xong, terminal in `Xe 1 ĐÃ TỚI ...`,
`Xe 2 ĐÃ TỚI ...` rồi hỏi lượt tiếp. Gõ `q` để thoát.

### Terminal 3: RViz
```bash
cd ~/robot_ws_3
source install/setup.bash
rviz2 -d tools/fleet.rviz
```
Hiện đầy đủ như Kịch bản 2, cho **từng xe** (nhóm `Xe 1 (agv1)`, `Xe 2 (agv2)` ở cột trái —
bỏ dấu tick để ẩn cả nhóm):

| Mục | Ý nghĩa |
|---|---|
| Mô hình xe | thân, bánh, LiDAR theo URDF |
| LaserScan | điểm quét **đỏ, cỡ 5** — phải trùng tường đen |
| Hạt AMCL | đám mũi tên quanh xe (xe 1 xanh, xe 2 cam); co gọn = định vị chắc |
| Local Costmap | vùng vật cản quanh xe |
| Global Costmap | tắt sẵn (hai xe chồng lên nhau rối) — tick để xem |
| Footprint, Đoạn đang chạy, Local Plan | khung xe, đoạn thẳng đang giao cho Nav2 |

Cộng thêm bản đồ, lưới sa bàn (điểm đỏ = bị chặn), đường còn lại và ô đang giữ của từng xe.
`tools/fleet_real.sh` tự chạy `tools/fleet_viz.py` để gộp cây TF hai xe (frame `agv1/...`,
`agv2/...`) cho một cửa sổ RViz.

### Chạy Thử Từng Bước (đã chạy thật 2026-10-10)

Lần đầu trong ngày nên chạy lại vài bước này. Mỗi bước đạt mới sang bước sau.

| Bước | Xe 1 | Xe 2 | Phải thấy | Kết quả thật |
|---|---|---|---|---|
| 1. Chỉ xe 1 | `3 2 d` | `3 5` Enter | xe 1 tiến 60 cm; xe 2 đứng yên | lệch 0.5 cm, 1.7° |
| 2. Chỉ xe 2 | `3 2` Enter | `3 4 t` | xe 2 tiến 60 cm; xe 1 đứng yên | lệch 0.4 cm, 0.9° |
| 3. Song song | `5 2 b` | `5 4 b` | cùng chạy, không ai chờ ai | lệch 1.2 / 0.4 cm |
| 4. Đổi chỗ hàng 5 | `5 4 d` | `5 2 t` | một xe né sang ô bên cạnh, xe kia qua | không va, gần nhất 60 cm |
| 5. Đường cắt nhau | `3 1 d` | `3 5 t` | một xe chờ trước chỗ giao | không va |
| 6. Đổi chỗ (như mô phỏng) | `3 5 t` | `3 1 d` | một xe né lên hàng 4 rồi vòng về | không va, gần nhất 59 cm |

> [!WARNING]
> **Độ chính xác phụ thuộc bản đồ.** Ngày 2026-10-10, phía **tây** sa bàn (quanh cột 1) có
> nhiều đồ đạc mới không có trên bản đồ (cách (3,1) ~48 cm, (4,1) ~26 cm, (5,1) ~45 cm):
> xe đứng ở cột 1 lệch 4–6 cm, phía đông thì < 1 cm. Dọn các vật đó, hoặc vẽ lại bản đồ
> (Kịch bản 1) rồi căn lại sa bàn, thì cột 1 cũng sẽ < 3 cm.

### Khi Xe Báo ESTOP
ESP32 tự dừng khẩn khi **bánh kẹt / mất encoder / hết pin** hoặc **bánh quay ngược chiều**,
và báo ngay về máy tính — Terminal 2 in đỏ:
```
!!! [agv1] ESTOP tren ESP32: Banh ket / mat encoder - xe dung han, bo lo trinh; ...
```
Xe đó bỏ lộ trình, xe kia tránh nó như xe đang đỗ. Sửa nguyên nhân rồi gỡ ESTOP (terminal mới):
```bash
ros2 topic pub --once /agv1/estop_reset std_msgs/msg/Empty     # hoặc tắt/bật nguồn xe
```
rồi nhập đích mới. Terminal 2 in `[agv1] da go ESTOP`.

### Kiểm Tra Động Cơ + Encoder (sau khi nạp firmware / đổi dây)
**Kê xe hở bánh**, tắt Terminal 2, rồi:
```bash
python3 tools/wheel_test.py agv1     # hoặc agv2
```
Bốn dòng tiến / lùi / quay trái / quay phải phải đều `DUNG` cho cả hai bánh. Tiến mà cả hai
bánh `NGUOC` → dây động cơ trái/phải nối chéo so với firmware (xe 1 nối thẳng, xe 2 nối chéo
— đổi trong file gốc firmware rồi `tools/agvctl fw split`).

### Hiệu Chỉnh Riêng Từng Xe
Mặc định cả hai xe dùng số đo của **xe 1**. Đo được số riêng thì khai báo khi bật:
```bash
WHEEL_R_agv2=0.0330 COAST_agv2=0.30 tools/fleet_real.sh up
```
- `WHEEL_R_agvN`: bán kính lăn bánh (m) — xe đi **thiếu** quãng đường thì tăng, **thừa** thì giảm.
- `COAST_agvN`: hệ số xoay tại chỗ (s) — xe xoay **vượt** góc thì tăng, **thiếu** thì giảm.

Terminal 1 in lại `Xe agv2: wheel_radius ... m, coast ...` để xác nhận.

### Tắt
```bash
cd ~/robot_ws_3
tools/fleet_real.sh down      # gửi lệnh dừng cho cả hai xe rồi tắt hết
```

> [!CAUTION]
> **Dừng khẩn:** Ctrl-C Terminal 2 (mất lệnh điều phối quá 1 giây thì từng xe tự dừng),
> hoặc `tools/fleet_real.sh down`, hoặc tắt công tắc nguồn xe.

> [!NOTE]
> - Log từng phần ở `/tmp/fleet_real/` (`agent_agv1.log`, `nav_agv2.log`, `robot_agv1.log`...).
> - Mỗi lần `up`/`down`, script dọn bộ nhớ chung DDS bị bỏ lại (`fastdds shm clean`).
> - Thử không cần xe (ESP32 giả, LiDAR giả dò tường trên bản đồ): chạy hai
>   `tools/fleet_fake_robot.py ... --esp32` rồi `AGENT=0 EMULATE=1 tools/fleet_real.sh up`.
