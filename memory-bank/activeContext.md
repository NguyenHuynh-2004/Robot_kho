# Active Context (cập nhật 2026-10-10)

## Đang làm
Triển khai **hai xe thật** (Kịch bản 4). Code, tài liệu và thử với ESP32 giả đã xong
(commit 1e5dec8). Bước tiếp theo là chạy trên hai xe thật.

## Đang chờ người dùng
1. Nạp firmware mới (có namespace `/agvN`) cho **cả hai xe**: xe 1 với `#define XE 1`, xe 2
   với `#define XE 2` (`D:\CTU\Ardunio ide\code\Xe_lidar\Xe_lidar.ino` đang để `XE 2`).
   Lưu ý: firmware cũ của xe 1 **không** có namespace → không chạy được với
   `bringup_real.launch.py xe:=1` mới cho tới khi nạp lại.
2. Đặt xe 1 ở (3,1) hướng Đông, xe 2 ở (3,5) hướng Tây, bật nguồn, rồi
   `tools/fleet_real.sh up` → `python3 traffic_manager.py agv1 agv2` → `rviz2 -d tools/fleet.rviz`.
3. Kiểm tra Nav2 (khởi động lại) với `always_send_full_costmap` ở chế độ 1 xe: nút đỏ phải
   chuyển xanh ~0.2–0.4 s sau khi bỏ vật cản.

## Gợi ý khi người dùng quay lại
- Sau lần chạy 2 xe thật đầu tiên: đọc `/tmp/fleet_real/*.log`; kiểm tra xe 2 xoay có vượt/thiếu
  góc (coast của xe 1) và quãng đường (wheel_radius của xe 1).
- Nếu có tiến trình "vô hình" → đếm tiến trình ROS (giới hạn dò tìm LOCALHOST).
- Thử không cần xe: hai `tools/fleet_fake_robot.py X Y YAW --esp32 --other /agvK/truth
  --ros-args -r __ns:=/agvN` rồi `ROS_DOMAIN_ID=89 AGENT=0 EMULATE=1 tools/fleet_real.sh up`.

## Quy tắc phiên làm việc
Nói tiếng Việt; người dùng tự nạp firmware; không gửi `/cmd_vel` tới xe thật khi chưa được
phép; không commit `wifi_config.h`; chỉ đẩy GitHub khi được bảo; làm xong từng giai đoạn chờ
người dùng nói "xong".
