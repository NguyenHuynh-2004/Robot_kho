// ============================================================================
//  TEST BNO055 TREN XE QUA WI-FI  (ESP32, I2C SDA GPIO21 / SCL GPIO22)
// ============================================================================
//  Sketch rieng, KHONG micro-ROS, KHONG chay dong co. Khoi tao BNO055 GIONG
//  HET esp32_agv_base.ino (dia chi 0x29, che do NDOF, thach anh ngoai), roi
//  gui so lieu QUA WI-FI: ESP32 mo cong TCP 2323, laptop ket noi vao de xem
//  va go lenh. (ESP32 lam may chu de laptop chu dong ket noi ra - khong vuong
//  tuong lua Windows / Hyper-V chan chieu vao WSL.)
//
//  CACH DUNG
//      1. Dat wifi_config.h (giong firmware chinh) CUNG THU MUC voi file .ino.
//      2. Nap sketch nay (giu nut BOOT luc Connecting...).
//      3. Rut cap USB cung duoc. Tren laptop (WSL):
//             cd ~/robot_ws_3 && python3 tools/bno055_wifi.py            # IP mac dinh 192.168.1.150
//             python3 tools/bno055_wifi.py 192.168.1.xxx                  # IP khac
//         IP cua xe in tren Serial 115200 luc khoi dong ("Wi-Fi OK, IP ...").
//      4. Go lenh (mot chu roi Enter):
//           h  huong dan            p  bat/tat in lien tuc (5 lan/giay)
//           z  dat yaw tuong doi=0  d  do TROI 60 s (de xe DUNG YEN)
//           r  do QUAY TAY: go r, xoay xe dung 90 do, go r lan nua
//           m  doi NDOF <-> IMUPLUS c  hieu chuan + loi he thong
//
//  DOC KET QUA
//      - acc |g| ~ 9.8 m/s2 khi dung yen; temp 20-45 C.
//      - Hieu chuan S,G,A,M tu 0 den 3. Gyro (G) len 3 sau vai giay dung yen.
//      - Quaternion toan 0 = BNO055 treo: TAT HAN NGUON XE roi bat lai.
//
//  Xong thi nap lai esp32_agv_base.ino.
// ============================================================================

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <stdarg.h>

#if __has_include("wifi_config.h")
#include "wifi_config.h"          // WIFI_SSID, WIFI_PASS (giong firmware chinh)
#else
#error "Thieu wifi_config.h: chep wifi_config.example.h thanh wifi_config.h vao cung thu muc voi file .ino"
#endif

const uint16_t TCP_PORT = 2323;

// Giong esp32_agv_base.ino
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;
uint8_t bno_addr = 0x29;           // tu doi sang 0x28 neu quet thay 0x28

WiFiServer server(TCP_PORT);
WiFiClient client;

Adafruit_BNO055 *bno = nullptr;
bool bno_ok = false;
adafruit_bno055_opmode_t mode = OPERATION_MODE_NDOF;
String i2c_report;                 // ket qua quet I2C, gui lai moi khi laptop ket noi

bool printing = true;
float yaw_zero = 0.0f;
uint32_t last_print = 0;

bool rot_running = false;
double rot_gyro_deg = 0.0;
float rot_q_start = 0.0f;
uint32_t rot_last_us = 0;

// Gui mot dong toi laptop (neu dang ket noi). Serial chi dung cho IP / loi Wi-Fi.
static void out(const char *fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (client && client.connected()) client.print(buf);
}

static float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// Yaw (do) tu quaternion, cung cong thuc voi wheel_odometry.py tren laptop
static float quat_yaw_deg() {
  imu::Quaternion q = bno->getQuat();
  float w = q.w(), x = q.x(), y = q.y(), z = q.z();
  return atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z)) * 180.0f / PI;
}

static bool quat_all_zero() {
  imu::Quaternion q = bno->getQuat();
  return fabsf(q.w()) + fabsf(q.x()) + fabsf(q.y()) + fabsf(q.z()) < 1e-3f;
}

static void i2c_scan() {
  i2c_report = "Quet I2C (SDA 21, SCL 22):\n";
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      char line[64];
      snprintf(line, sizeof(line), "  thay thiet bi o 0x%02X%s\n", a,
               (a == 0x28 || a == 0x29) ? "  <- BNO055" : "");
      i2c_report += line;
      if (a == 0x28 || a == 0x29) bno_addr = a;
      n++;
    }
  }
  if (n == 0) i2c_report += "  KHONG thay thiet bi nao - kiem tra 4 day: 3V3, GND, SDA->21, SCL->22.\n";
}

static void print_calib() {
  uint8_t s, g, a, m;
  bno->getCalibration(&s, &g, &a, &m);
  uint8_t st, test, err;
  bno->getSystemStatus(&st, &test, &err);
  out("Hieu chuan S,G,A,M = %u,%u,%u,%u | trang thai %u, tu kiem tra 0x%02X (0x0F = dat), loi %u\n",
      s, g, a, m, st, test, err);
}

static void set_mode(adafruit_bno055_opmode_t m) {
  bno->setMode(m);
  mode = m;
  delay(30);
  out("Che do: %s\n", m == OPERATION_MODE_NDOF ? "NDOF (co la ban tu - giong firmware)"
                                               : "IMUPLUS (khong la ban tu)");
}

static void help() {
  out("Lenh: h huong dan | p bat/tat in | z dat 0 | d do troi 60 s | "
      "r do quay tay | m doi NDOF/IMUPLUS | c hieu chuan\n");
}

// Do troi: dung yen 60 s, gui moi 10 s
static void drift_test() {
  out("DO TROI 60 s - DE XE DUNG YEN, KHONG CHAM VAO XE...\n");
  delay(1000);
  float y0 = quat_yaw_deg();
  double gsum = 0.0;
  uint32_t n = 0, t0 = millis(), tnext = t0 + 10000;
  while (millis() - t0 < 60000) {
    gsum += bno->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE).z();
    n++;
    if (millis() >= tnext) {
      out("  %2lu s: yaw troi %+6.2f do\n", (unsigned long)((millis() - t0) / 1000),
          wrap180(quat_yaw_deg() - y0));
      tnext += 10000;
    }
    delay(20);
  }
  float drift = wrap180(quat_yaw_deg() - y0);
  out("KET QUA: quaternion troi %+.2f do/phut | gyro z trung binh (bias) %+.3f do/s (%lu mau)\n",
      drift, gsum / n, (unsigned long)n);
  out("  (|troi| < 1 do/phut la tot; firmware tu tru bias gyro do luc khoi dong)\n");
}

static void connect_wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Ket noi Wi-Fi \"%s\"", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nWi-Fi OK, IP %s - tren laptop: python3 tools/bno055_wifi.py %s\n",
                WiFi.localIP().toString().c_str(), WiFi.localIP().toString().c_str());
}

static void greet() {
  out("\n=== TEST BNO055 QUA WI-FI (xe %s) ===\n", WiFi.localIP().toString().c_str());
  out("%s", i2c_report.c_str());
  if (!bno_ok) {
    out("KHONG khoi tao duoc BNO055 o 0x%02X. Kiem tra day, chan ADR, nguon 3V3.\n", bno_addr);
    out("Neu truoc do chay binh thuong: TAT HAN NGUON XE vai giay roi bat lai.\n");
    return;
  }
  out("BNO055 o 0x%02X, che do %s\n", bno_addr, mode == OPERATION_MODE_NDOF ? "NDOF" : "IMUPLUS");
  print_calib();
  help();
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== TEST BNO055 QUA WI-FI ===");
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  i2c_scan();
  Serial.print(i2c_report);
  bno = new Adafruit_BNO055(55, bno_addr, &Wire);
  bno_ok = bno->begin();
  if (bno_ok) {
    bno->setExtCrystalUse(true);
    delay(1000);
    bno->setMode(OPERATION_MODE_NDOF);
    yaw_zero = quat_yaw_deg();
  } else {
    Serial.println("KHONG khoi tao duoc BNO055 - van mo Wi-Fi de bao loi qua mang.");
  }
  connect_wifi();
  server.begin();
  server.setNoDelay(true);
}

void loop() {
  // Wi-Fi rot thi ket noi lai
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Mat Wi-Fi, ket noi lai...");
    connect_wifi();
  }
  // Laptop ket noi moi (chi giu mot ket noi)
  if (!client || !client.connected()) {
    WiFiClient c = server.available();
    if (c) {
      client = c;
      client.setNoDelay(true);
      Serial.printf("Laptop %s da ket noi\n", client.remoteIP().toString().c_str());
      greet();
    }
  }
  if (!bno_ok) {
    delay(50);
    return;
  }

  // Lenh tu laptop
  while (client && client.connected() && client.available()) {
    char ch = tolower(client.read());
    switch (ch) {
      case 'h': help(); break;
      case 'p': printing = !printing; out("In lien tuc: %s\n", printing ? "BAT" : "TAT"); break;
      case 'z': yaw_zero = quat_yaw_deg(); out("Da dat yaw tuong doi = 0\n"); break;
      case 'd': drift_test(); break;
      case 'm': set_mode(mode == OPERATION_MODE_NDOF ? OPERATION_MODE_IMUPLUS : OPERATION_MODE_NDOF); break;
      case 'c': print_calib(); break;
      case 'r':
        if (!rot_running) {
          rot_running = true;
          rot_gyro_deg = 0.0;
          rot_q_start = quat_yaw_deg();
          rot_last_us = micros();
          out("DO QUAY: xoay xe bang tay (vd dung 90 do theo canh gach), xong go r lan nua.\n");
        } else {
          rot_running = false;
          float q_deg = wrap180(quat_yaw_deg() - rot_q_start);
          out("KET QUA QUAY: quaternion %+.1f do | tich phan gyro %+.1f do | lech %.1f do\n",
              q_deg, rot_gyro_deg, fabs(q_deg - rot_gyro_deg));
          out("  (xoay 90 do ma ca hai ~90 va lech < 3 do la tot)\n");
        }
        break;
      default: break;
    }
  }

  // Tich phan gyro z khi dang do quay (50 Hz)
  static uint32_t last_int = 0;
  if (rot_running && millis() - last_int >= 20) {
    last_int = millis();
    uint32_t now = micros();
    float gz = bno->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE).z();   // do/s
    rot_gyro_deg += gz * (now - rot_last_us) * 1e-6;
    rot_last_us = now;
  }

  // Gui 5 lan/giay khi co laptop ket noi
  if (printing && client && client.connected() && millis() - last_print >= 200) {
    last_print = millis();
    if (quat_all_zero()) {
      out("CANH BAO: quaternion toan 0 - BNO055 treo. TAT HAN NGUON XE roi bat lai.\n");
      return;
    }
    sensors_event_t e;
    bno->getEvent(&e);
    imu::Vector<3> g = bno->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> a = bno->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    float qy = quat_yaw_deg();
    uint8_t s, gc, ac, mc;
    bno->getCalibration(&s, &gc, &ac, &mc);
    out("yaw %+7.2f (tuong doi %+7.2f) pitch %+6.1f roll %+6.1f | gyro z %+7.2f do/s | "
        "acc |g| %5.2f | %2d C | cal %u%u%u%u%s\n",
        qy, wrap180(qy - yaw_zero), e.orientation.y, e.orientation.z, g.z(),
        sqrtf(a.x() * a.x() + a.y() * a.y() + a.z() * a.z()), bno->getTemp(),
        s, gc, ac, mc, rot_running ? " | DANG DO QUAY" : "");
  }
}
