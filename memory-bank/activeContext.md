# Active Context (cập nhật 2026-10-10 tối)

## Trạng thái
**Hai xe thật chạy được theo logic mô phỏng** (2026-10-10): 6 bước thử (từng xe → song song →
đổi chỗ hàng 5 → đường cắt nhau → đổi chỗ như mô phỏng) đều phối hợp đúng, không va (gần
nhất 59–60 cm). Người dùng xác nhận OK; hướng dẫn Kịch bản 4 đã viết lại để người dùng tự chạy.

Phần cứng đã chốt:
- Cả hai xe dùng **mạch in 2026-10-05** (encoder 23/18/19/17, PWM LiDAR 4).
- Dây động cơ: **xe 1 nối thẳng** (27/26/25 trái), **xe 2 nối chéo** (33/32/14 trái).
- LiDAR: xe 1 ×1.0, xe 2 ×1.062. Gyro xe 1 cùng chiều encoder.
- Firmware **tách 2 file**: `Xe_1\Xe_1.ino`, `Xe_2\Xe_2.ino` (sinh từ file gốc bằng
  `tools/agvctl fw split`). Có `/agvN/diag` + sự kiện ESTOP gửi ngay + `/agvN/estop_reset`.
- URDF: `base_footprint` tại **tâm trục bánh** (xoay tại chỗ không trôi vị trí).

## Vấn đề còn mở
- **Xe 2 BROWNOUT (~18:20 ngày 2026-10-10)**: ESP32 xe 2 tự khởi động lại vì sụt áp, sau đó
  `joint_states` (publisher reliable) gửi lỗi mãi (`pub_fail` ~50/s) → không odom/TF. Cần sạc pin,
  bật lại xe, `fleet_real.sh down/up`. Brownout lặp lại → kiểm tra pin / tụ nguồn ESP32.
- Costmap: đã đổi cấu hình + `fleet_agent` (decay 1 s) nhưng **chưa thử với cả stack mới**
  (đo trên agv1 bằng script: ô kẹt 0, vật thật không chớp).
- Phía **tây** sa bàn (cột 1) có đồ đạc mới không có trên bản đồ → xe ở cột 1 lệch 4–6 cm
  (phía đông < 1 cm). Cần dọn hoặc **vẽ lại bản đồ + căn lại sa bàn**.
- Chế độ 1 xe chưa chạy lại sau khi đổi URDF (base_footprint) — nên thử một lần.
- Xe 2 vẫn dùng `wheel_radius`/`coast` của xe 1 (chạy thật thấy ổn).

## Đo hàng bằng camera — ĐÃ CHẠY MÔ PHỎNG (2026-10-12), chưa làm xe thật
Chỉ đo **rộng × cao**; camera nhìn thẳng (cao 11 cm, trước trục 6 cm), **Z + mặt phẳng mặt hàng
từ LiDAR** (trung vị từng tia qua 10 vòng → RANSAC đường thẳng); xe KHÔNG chở hàng.
- `cargo_measure.py` (thuật toán + đo 1 lần), `cargo_task.py` (lệnh "1 3": tới (5,3) quay Bắc →
  đo → `cargo.yaml classes` → `/fleet/goal` về (1,k); ô có xe đã giao đỗ → xe đó lùi lên hàng 2),
  `tools/sim_cargo.py` (hộp + camera ảo có méo/nhiễu/JPEG), `fleet_fake_robot` thấy hộp
  (`FAKE_LIDAR_NOISE`, `FAKE_RAYS`). Hướng dẫn: HUONG_DAN_CHAY Kịch bản 3 → "Đo Hàng Bằng Camera".
- Kết quả mô phỏng: sai ≤ 3,5 mm (hàng 9–30 cm, xoay ≤ 21°, xe lệch 3 cm/3°); hàng chạm mép ảnh
  → báo lỗi, không giao. Giới hạn: rộng ≤ ~33 cm, cao 11–25 cm, hàng cách tâm ô 35 cm.
- Xe thật còn thiếu: ESP32-CAM + nguồn 5 V riêng, firmware chụp (HTTP `/capture` → node phát
  `/agvN/camera/image_raw`), hiệu chuẩn (thay `camera.*` trong cargo.yaml), đo vị trí lắp camera.

## Gợi ý khi người dùng quay lại
- Chạy: `tools/fleet_real.sh up` → `python3 tools/fleet_localize.py` → `traffic_manager.py`.
- Lỗi xe không chạy: xem `/agvN/diag` (ESTOP + lý do), `tools/wheel_test.py agvN` (kê bánh).
- Không cho xe chạy khi người dùng chưa cho phép trong phiên.

## Quy tắc phiên làm việc
Nói tiếng Việt; người dùng tự nạp firmware; không gửi `/cmd_vel` tới xe thật khi chưa được
phép; không commit `wifi_config.h` và `nhiem_vu.txt`; chỉ đẩy GitHub khi được bảo.
