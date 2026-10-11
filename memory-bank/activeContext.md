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

## Ý tưởng tiếp theo (chưa làm, 2026-10-11)
Đo kích thước hàng ở hàng 5 rồi xe chạy về ô tương ứng ở hàng 1 (xe KHÔNG chở hàng).
Người dùng chốt: **camera nhìn thẳng** (ESP32-CAM riêng, nguồn 5 V riêng), **Z lấy từ LiDAR**
(khớp đường thẳng mặt trước hàng, ±15° phía trước). Camera nhìn thẳng chỉ cho rộng × cao;
chiều sâu cần LiDAR nhìn chéo từ ô bên cạnh — **chờ người dùng chốt cần đo chiều nào**.
Ràng buộc: hàng cao > ~10 cm (LiDAR ở 7,5 cm), cách nút hàng 5 khoảng 30–40 cm (gần hơn
costmap chặn nút), nền trơn; hàng 1 chỉ dùng (1,1)–(1,4) vì (1,5) bị bỏ.
Luồng: lệnh `do <xe> 5 <c>` trong traffic_manager → tới (5,c) hướng Bắc → LiDAR Z + ảnh →
`rộng = w_px·Z/fx` → bảng phân loại → goal (1,k). Dự kiến ±3–5 mm.

## Gợi ý khi người dùng quay lại
- Chạy: `tools/fleet_real.sh up` → `python3 tools/fleet_localize.py` → `traffic_manager.py`.
- Lỗi xe không chạy: xem `/agvN/diag` (ESTOP + lý do), `tools/wheel_test.py agvN` (kê bánh).
- Không cho xe chạy khi người dùng chưa cho phép trong phiên.

## Quy tắc phiên làm việc
Nói tiếng Việt; người dùng tự nạp firmware; không gửi `/cmd_vel` tới xe thật khi chưa được
phép; không commit `wifi_config.h` và `nhiem_vu.txt`; chỉ đẩy GitHub khi được bảo.
