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

## Đo hàng bằng camera — code xe thật XONG (2026-10-13), CHỜ phần cứng
Chỉ đo **rộng × cao**; camera nhìn thẳng, **Z + mặt phẳng mặt hàng từ LiDAR**; xe KHÔNG chở hàng.
**Lấy hàng phía Đông ô (2..5, 5)** (xe quay Đông; (1,5) bị bỏ), **trả về hàng 1** cột theo loại
(`cargo.yaml place.*`, `classes`). Lệnh `cargo_task.py`: "<xe> <hàng>" (vd "1 3" → (3,5)).
- ESP32-CAM: `firmware/esp32_cam` (`#define XE`, IP tĩnh .200/.201, `/capture` :80, `/stream` :81,
  `/status`), biên dịch OK (board esp32cam); đã chép `D:\CTU\Ardunio ide\code\Cam_1`, `Cam_2`
  (+ wifi_config.h). Người dùng có sẵn `CameraWebServer`, `code_camera` (QQVGA) — không dùng.
- `tools/esp32_cam_node.py` (tham khảo esp32_cam_node.py người dùng gửi: luồng riêng giữ ảnh mới
  nhất; thêm mode capture mặc định, tự nối lại, rotate) — `fleet_real.sh up` tự bật (`CAM=0` tắt).
  Thử với ESP32-CAM giả: capture 5/s, stream 12/s, mất/nối lại OK.
- `tools/cam_calib.py agvN` → `config/camera_agvN.yaml` (đè `camera:`; x,y,z đo tay). Thử bàn cờ
  giả: fx 492,1/492,8, RMS 0,28. `cargo_task.py --sim` bỏ qua file hiệu chuẩn.
- Mô phỏng bố trí mới: 4 hàng sai ≤ 1 mm, về đúng (1,1)–(1,4).
- **Chờ người dùng:** lắp ESP32-CAM + nguồn 5 V riêng, nạp Cam_1/Cam_2, hiệu chuẩn, đo x,y,z.
- Bẫy: thử phát hiện node bằng `timeout ros2 run ...` cho kết quả sai (chập chờn) — chạy
  thẳng python, chờ ≥ 5 s.

## Gợi ý khi người dùng quay lại
- Chạy: `tools/fleet_real.sh up` → `python3 tools/fleet_localize.py` → `traffic_manager.py`.
- Lỗi xe không chạy: xem `/agvN/diag` (ESTOP + lý do), `tools/wheel_test.py agvN` (kê bánh).
- Không cho xe chạy khi người dùng chưa cho phép trong phiên.

## Quy tắc phiên làm việc
Nói tiếng Việt; người dùng tự nạp firmware; không gửi `/cmd_vel` tới xe thật khi chưa được
phép; không commit `wifi_config.h` và `nhiem_vu.txt`; chỉ đẩy GitHub khi được bảo.
