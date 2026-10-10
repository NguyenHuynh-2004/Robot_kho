# Progress

## Đã xong (mới nhất trước)
| Ngày | Việc | Commit |
|---|---|---|
| 2026-10-10 | **Hai xe thật**: micro-ROS gắn ID xe (`/agvN`), `tools/fleet_real.sh`, `fleet_real_robot.launch.py`, Nav2 gộp container, `ns_relay.py` cho chế độ 1 xe, `auto_initial_pose --ns`, `fleet_fake_robot --esp32` (ESP32 giả, LiDAR dò tường bản đồ), Kịch bản 4 trong tài liệu. **Mới thử bằng ESP32 giả**, chưa chạy trên 2 xe thật | 1e5dec8 |
| 2026-10-10 | Local costmap luôn phát bản đầy đủ 5 Hz (nút đỏ không xoá khi xe đứng yên) | bffb85c |
| 2026-10-10 | Xe 2 không nhận lệnh: dây động cơ L/R nối chéo trên mạch mới → đổi nhóm chân trong `XE 2` | d0a9406 |
| 2026-10-10 | `#define XE`: xe 1 giữ nguyên, xe 2 mạch mới + LiDAR ×1.062 | da5d42a, 1bc41d0 |
| 2026-10-10 | RViz/Gazebo vẽ bằng GPU (`GALLIUM_DRIVER=d3d12`); sửa shader RViz; ẩn COPY MODE | 601b5b0, 6a3a454 |
| 2026-10-06 | Gazebo trong mô phỏng 2 xe: xem 3D + kéo trụ đỏ làm vật cản | 1c9a36e, 901833d |
| 2026-10-06 | Sketch test BNO055 qua Wi-Fi TCP 2323 | b8a511c, 23c4eea |
| 2026-10-06 | Tài liệu viết lại theo kịch bản; tách `SA_BAN_VA_TAI_LIEU.md` | ee1827e, 2ff7f0c |
| 2026-10-05 | Điều phối 2 xe (mô phỏng): giữ chỗ, ưu tiên gần đích, gỡ kẹt; firmware chân mạch in 2026-10-05 | 679f763, c3256a7 |
| trước đó | 1 xe thật trên sa bàn: xoay tại chỗ bằng gyro + coast, PID Kf 3.0, goal 2 cm; đẩy GitHub "1 Xe" | da0278b |

GitHub: `main` đang **đi trước `origin/main` 18 commit** (chưa đẩy; chỉ đẩy khi người dùng bảo).

## Số đo
- **Xe 1 thật (2026-10-03):** 9/9 lần xoay ≤ 2.8° trong 4–6 s; tới đích lệch 0.4–1.6 cm;
  lệch cạnh khi chạy ≤ 0.6 cm trung bình (theo AMCL). LiDAR lệch tâm quay ~1.1 cm.
- **Xe 2 thật (2026-10-10):** LiDAR góc đúng (1.1°), khoảng cách thiếu 6.2% → bù 1.062, sau
  bù khớp 1.0014. Thử kê bánh tiến/lùi/trái/phải đều đúng chiều sau khi đổi chân động cơ.
  Xoay quanh 90° đo ω max 56°/s. **Chưa** chạy trọn sa bàn trên sàn.
- **Mô phỏng 2 xe (2026-10-10, sau khi gộp container):** swap, cross, headon, parked, prio,
  follow đều đạt; khoảng cách tâm–tâm nhỏ nhất ≥ 57 cm; tới đích lệch ≤ 2.1 cm. (Cross từng
  hỏng một lần vì lập đường lần đầu không thử lại → đã sửa.)
- **2 xe "thật" với ESP32 giả (domain 89):** đổi chỗ agv1 (3,1)↔agv2 (3,5) đạt, lệch đích
  2.9–3.4 cm (AMCL), hướng ≤ 3.4°; 18 tiến trình ROS; mỗi agent ~9% CPU.

## Vấn đề còn mở
- **Chạy 2 xe thật lần đầu** (Kịch bản 4) — đang chờ người dùng nạp firmware có namespace
  cho cả 2 xe.
- Xe 2: `wheel_radius` (0.03281) và `coast` (0.36) đang dùng số đo của xe 1 → có thể cần đo
  lại riêng (`tools/check_odom_scale.py`, phân tích xoay).
- Xe trôi song song vạch dù RViz báo bám (nghi sai lệch tịnh tiến lưới ↔ băng keo) — cần
  người dùng đo bằng thước; chưa có số.
- `firmware/encoder_test` còn chân cũ.
- Nút (1,5) bị bỏ vì sát vật cản trên bản đồ (14 cm < 25 cm).
- Mạch in xe 2: J2_1 (3V3 cho encoder/BNO055) từng chưa nối — đã hỏi, chưa xác nhận.

## Lỗi đã sửa — đừng lặp lại
- Pending model xoay tại chỗ (tính cả lệnh xe không nhúc nhích) → dùng gyro + coast.
- `/odom` ω có gai → điều khiển xoay theo gyro (`rot_w()`).
- Goal checker dừng ở 2.6–2.9 cm → `xy_goal_tolerance 0.02`.
- `navigation_launch.py` không đẩy namespace → `fleet_nav.launch.py` dùng `PushROSNamespace`.
- Costmap đa xe nghe `/agvN/local_costmap/scan` → topic tuyệt đối từng xe.
- LiDAR giả không tường → bóng ma không xoá → `inf_is_valid` (chỉ mô phỏng).
- `route_id` trùng giữa các lần chạy traffic_manager → id theo thời gian.
- Agent trùng tên chạy song song giành goal → `fleet_sim.sh down` pkill theo mẫu.
- Nav2 2 xe bật song song mất trả lời lifecycle → bật tuần tự + thử lại.
- `MultiThreadedExecutor` ăn CPU; DDS LOCALHOST giới hạn tiến trình; container thiếu file
  tham số → xem techContext "Bẫy".
