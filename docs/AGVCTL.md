# agvctl — chạy và đọc lỗi đội xe không cần mở terminal

Mục tiêu: người dùng **chỉ bật nguồn xe**. Mọi thứ còn lại (bật hệ, chờ xe, Nav2, điều phối,
đọc lỗi ROS, đọc Serial ESP32, biên dịch firmware) chạy bằng một lệnh. Lệnh này dùng được
từ Claude Code hoặc từ script.

```
tools/agvctl doctor      # kiểm tra môi trường
tools/agvctl up          # bật hệ chạy nền, chờ xe bật nguồn
tools/agvctl status      # trạng thái từng xe
tools/agvctl errors      # lỗi gom từ mọi node
tools/agvctl down        # tắt hết
```

## Các thành phần

| Thành phần | Vai trò |
|---|---|
| `tools/agvctl` | Một cửa: `doctor`, `up`, `down`, `status`, `errors`, `events`, `logs`, `goal`, `stop`, `localize`, `serial`, `fw build`, `fw copy` |
| `tools/fleet_monitor.py` | Một node ROS (một tiến trình) gom **/rosout** của mọi node (Nav2, AMCL, fleet_agent, traffic_manager...), nhịp `/agvN/scan` và `joint_states`, `/agvN/diag`, AMCL, `/fleet/status`, và danh sách node đang thiếu. Ghi ra `status.json` (mỗi giây), `errors.log`, `events.log` |
| `/agvN/diag` (firmware) | Mỗi giây ESP32 gửi một chuỗi JSON qua Wi-Fi: ESTOP và lý do, RSSI, lý do khởi động lại (BROWNOUT/WATCHDOG...), RPM/PWM, nhịp publish, heap. **Đọc được khi xe chạy pin**, không cần USB |
| `/agvN/estop_reset` (firmware) | Gửi `std_msgs/Empty` để gỡ ESTOP mà không phải tắt/bật nguồn. Lệnh chỉ có tác dụng khi hai bánh đã dừng hẳn |
| `.mcp.json` → server MCP `ros` | Claude Code gọi thẳng ROS (topic, service, param, node) qua rosbridge `ws://127.0.0.1:9090` (`ros-mcp` 3.1.2) |
| `arduino-cli.exe` của Arduino IDE | `agvctl fw build N` biên dịch firmware từ WSL, in lỗi và kích thước. Không cần mở IDE |
| PowerShell (Windows) | `agvctl serial` đọc Serial ESP32 cắm USB. Tự tìm cổng theo chip CP210x/CH340/CH9102/FTDI |

Log của mỗi lần `up` nằm ở `~/agv_logs/run-<giờ>/`, còn `~/agv_logs/latest` trỏ tới lần mới
nhất:
`bringup.log`, `status.json`, `errors.log`, `events.log`, `monitor.log`, `micro_ros_agent.log`,
`rosbridge.log`, `traffic_manager.log`, `fleet/` (log của `fleet_real.sh`: AMCL, Nav2, agent
từng xe). Serial lưu ở `~/agv_logs/serial/`, log biên dịch ở `~/agv_logs/firmware/`.

## Cài một lần

1. rosbridge (cần sudo), dùng cho MCP `ros`:
   ```bash
   sudo apt install -y ros-jazzy-rosbridge-server
   ```
2. `uv` đã cài sẵn ở `~/.local/bin` (không cần sudo). `.mcp.json` nằm ở gốc repo. Mở lại
   phiên Claude Code và chấp nhận server `ros`.
3. Nạp firmware mới (có namespace `/agvN` và `/diag`) cho **cả hai xe**:
   ```bash
   tools/agvctl fw build 1
   tools/agvctl fw copy 1
   ```
   Lệnh `fw copy` chép sang `D:\CTU\Ardunio ide\code\Xe_lidar\Xe_lidar.ino` (bản cũ được lưu
   ở `~/agv_logs/firmware/`). Sau đó mở Arduino IDE, giữ BOOT và nạp. Làm lại tương tự với
   `2` cho xe 2.

## Chạy hằng ngày

1. Đặt xe 1 ở (3,1) hướng Đông, xe 2 ở (3,5) hướng Tây, hoặc khai báo chỗ khác bằng
   `AGV1="hàng cột độ" AGV2=...`.
2. `tools/agvctl up`: lệnh trả về ngay. Hệ chờ **không giới hạn** tới khi đủ xe có `/agvN/scan`.
3. Bật nguồn xe và **để yên 2 giây** cho BNO055 đo bias. Khi đủ xe, hệ tự chạy
   `fleet_real.sh` (AMCL, Nav2 từng xe, fleet_agent) rồi `traffic_manager` (không cần tty).
4. Giao đích: `tools/agvctl goal "pair 1 3 5 d ; 2 3 1 t" --go`. Trên domain 0 (xe thật) lệnh
   **bắt buộc có `--go`**, để tránh vô tình cho xe chạy.
5. Dừng khẩn: `tools/agvctl stop`. Tắt hệ: `tools/agvctl down`.

Thử toàn bộ không cần xe (ESP32 giả, domain riêng):
```bash
ROS_DOMAIN_ID=89 tools/agvctl up --emulate
ROS_DOMAIN_ID=89 tools/agvctl goal "pair 1 4 2 b ; 2 2 4 n"
ROS_DOMAIN_ID=89 tools/agvctl down
```

## Đọc trạng thái và lỗi

`agvctl status` cho biết với từng xe:
- **KẾT NỐI / MẤT KẾT NỐI**: có `/scan` trong 2 s gần nhất hay không;
- nhịp `scan` và `joint_states` (thấp hơn 6 / 40 Hz trong 2 s liên tiếp thì báo Wi-Fi yếu);
- tỉ lệ tia LiDAR hợp lệ;
- dòng `ESP32:` lấy từ `/diag` (ESTOP, RSSI, lý do reset...);
- vị trí theo ô (hàng, cột), độ lệch so với tâm ô (cm) và lệch hướng (độ), dùng để so với
  tiêu chí ≤ 3 cm / ≤ 3°;
- độ lệch chuẩn AMCL; trạng thái agent; các node còn thiếu.

Monitor ghi một dòng vào `events.log` mỗi khi trạng thái đổi: ESP32 kết nối hoặc mất, ESTOP,
"có lệnh chạy 2 s mà bánh đứng yên" (nghi ESTOP, kẹt bánh hoặc hết pin; với firmware cũ không
có `/diag`), định vị kém, node mất hoặc có lại. `errors.log` lấy mọi WARN/ERROR/FATAL từ
`/rosout`; một dòng lặp lại trong 30 s chỉ ghi một lần, kèm số lần lặp.

## Ghi chú

- Monitor là **một** tiến trình, tốn khoảng 15% một nhân. Nó không nghe `imu/data`, vì mỗi
  callback rclpy tốn khoảng 1 ms; nhịp IMU đã có trong `/diag`.
- COM3 trên laptop này là thiết bị Lenovo gắn sẵn (VID_17EF), **không phải ESP32**. ESP32 cắm
  USB sẽ hiện ra ở một cổng COM khác.
- Firmware hiện có kích thước 1.07 MB. Nếu đổi partition sang "Minimal SPIFFS (1.9 MB APP with
  OTA)" thì sau này nạp được qua Wi-Fi (OTA), không cần cắm USB. Phương án này chưa làm.
