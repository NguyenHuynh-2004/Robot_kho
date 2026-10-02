// MAU cau hinh mang cho esp32_agv_base.ino.
//
// Chep file nay thanh wifi_config.h, dat CUNG THU MUC voi file .ino (ca thu
// muc sketch cua Arduino IDE), roi dien gia tri that. wifi_config.h nam trong
// .gitignore: mat khau Wi-Fi khong bao gio vao git.
#pragma once

#define WIFI_SSID   "TEN_WIFI"
#define WIFI_PASS   "MAT_KHAU_WIFI"
#define AGENT_IP    "192.168.1.21"     // IP laptop dang chay micro_ros_agent (hostname -I)
