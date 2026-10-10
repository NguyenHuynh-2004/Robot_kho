# Memory Bank — robot_ws_3

Ngữ cảnh dự án cho người và trợ lý AI bắt đầu một phiên làm việc mới. **Đọc theo thứ
tự**, dừng khi đủ:

| File | Nội dung | Khi nào đọc |
|---|---|---|
| [COMPACT.md](COMPACT.md) | Toàn bộ ngữ cảnh cốt lõi trong một trang | **Luôn đọc trước** |
| [projectbrief.md](projectbrief.md) | Mục tiêu, phạm vi, tiêu chí đạt | Khi cần hiểu "làm để làm gì" |
| [techContext.md](techContext.md) | Phần cứng, phần mềm, môi trường WSL, build, bẫy hay gặp | Trước khi chạy lệnh / sửa firmware |
| [systemPatterns.md](systemPatterns.md) | Kiến trúc, topic/TF, điều phối 2 xe, quyết định thiết kế | Trước khi sửa code |
| [activeContext.md](activeContext.md) | Đang làm gì, việc đang chờ người dùng | Đầu mỗi phiên |
| [progress.md](progress.md) | Đã xong gì, số đo, vấn đề còn mở, lịch sử lỗi đã sửa | Khi cần chi tiết / tránh lặp lỗi cũ |

Tài liệu cho người dùng nằm ở `docs/`: [HUONG_DAN_CHAY.md](../docs/HUONG_DAN_CHAY.md)
(các kịch bản chạy) và [SA_BAN_VA_TAI_LIEU.md](../docs/SA_BAN_VA_TAI_LIEU.md) (sa bàn,
tra sự cố, nối dây, thuật toán, quy tắc 2 xe, nạp firmware).

## Quy ước cập nhật

- Cuối mỗi phiên: sửa **activeContext.md** (đang làm / đang chờ) và **progress.md**
  (thêm mục đã xong, số đo mới, lỗi mới đã sửa). Ghi ngày tuyệt đối (YYYY-MM-DD).
- Thay đổi kiến trúc, topic, tham số quan trọng → sửa **systemPatterns.md** /
  **techContext.md** và rút gọn vào **COMPACT.md** (giữ COMPACT ≤ ~150 dòng).
- Không ghi mật khẩu Wi-Fi hay nội dung `wifi_config.h` vào đây.
