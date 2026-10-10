# Active Context (cập nhật 2026-10-10 chiều)

## Đang làm
Chạy **hai xe thật** (Kịch bản 4). Người dùng đã chạy thành công **một xe thật** (nhiều khả
năng xe 2, firmware namespace bản `1e5dec8` — đúng bản đang nằm ở
`D:\CTU\Ardunio ide\code\Xe_lidar\Xe_lidar.ino`, `XE 2`, chưa có `/diag`).

Đã xong trong phiên này (commit `a4af8ee` + tài liệu):
- `fleet_real.sh`: hiệu chỉnh riêng từng xe `WHEEL_R_agvN`, `COAST_agvN`; `fastdds shm clean`
  khi up/down (cổng /dev/shm bị `kill -9` bỏ lại làm container Nav2 "vô hình").
- Commit khối `agvctl`, `fleet_monitor.py`, `.mcp.json`, firmware `/diag` + `estop_reset`
  (firmware này **chưa chạy trên xe thật**).
- Kịch bản 4 có **6 bước thử lần đầu** (đứng yên → từng xe → hai xe song song → nhường
  đường → đường chéo → đổi chỗ). Đã chạy đúng 6 bước trên 2 ESP32 giả (domain 89): đều tới
  đích, không va.

## Đang chờ người dùng
1. Firmware: xe nào đã chạy được với `bringup_real.launch.py xe:=N` thì giữ nguyên. Xe còn lại
   nạp `Xe_lidar.ino` (bản 1e5dec8) với `#define XE` tương ứng — **không cần** bản `/diag`.
2. Chạy Kịch bản 4 theo 6 bước, gửi lại log `/tmp/fleet_real/*.log` nếu có lỗi.
3. (Tuỳ chọn) `sudo apt install -y ros-jazzy-rosbridge-server` cho MCP `ros` (lần này MCP
   `ros` không kết nối được).

## Gợi ý khi người dùng quay lại
- Lỗi "khong thay Nav2" ở một agent → đếm tiến trình ROS, kiểm tra `/dev/shm`, `fastdds shm clean`.
- Xe 2 xoay vượt/thiếu → `COAST_agv2`; đi thiếu/thừa quãng → `WHEEL_R_agv2`.
- Không cho xe chạy (`agvctl goal --go`, `/cmd_vel`) khi người dùng chưa cho phép trong phiên.
- Thử không cần xe: `steps_test` = hai `tools/fleet_fake_robot.py ... --esp32` +
  `ROS_DOMAIN_ID=89 AGENT=0 EMULATE=1 tools/fleet_real.sh up` + `/fleet/goal "pair ..."`.

## Quy tắc phiên làm việc
Nói tiếng Việt; người dùng tự nạp firmware; không gửi `/cmd_vel` tới xe thật khi chưa được
phép; không commit `wifi_config.h` và `nhiem_vu.txt`; chỉ đẩy GitHub khi được bảo; làm xong
từng giai đoạn chờ người dùng nói "xong".
