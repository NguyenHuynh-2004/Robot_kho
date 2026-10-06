// ============================================================================
//  TEST BNO055 TREN XE  (ESP32, I2C SDA GPIO21 / SCL GPIO22 - nhu mach in)
// ============================================================================
//  Sketch rieng, KHONG micro-ROS, KHONG Wi-Fi, KHONG chay dong co. Khoi tao
//  BNO055 GIONG HET esp32_agv_base.ino (dia chi 0x29, che do NDOF mac dinh,
//  thach anh ngoai) roi in so lieu va lam vai phep do.
//
//  CACH DUNG
//      1. Nap sketch nay (giu nut BOOT luc Connecting...).
//      2. Serial Monitor 115200 baud, dong "Newline" hoac "Both NL & CR".
//      3. Doc ket qua quet I2C luc khoi dong: phai thay 0x28 hoac 0x29.
//      4. Go lenh (mot chu roi Enter):
//           h  - huong dan
//           p  - bat/tat in lien tuc 5 lan/giay (mac dinh bat)
//           z  - lay huong hien tai lam 0 (cot "yaw tuong doi")
//           d  - do TROI 60 s: de xe DUNG YEN, in do troi do/phut va bias gyro
//           r  - do QUAY TAY: go r, xoay xe dung 90 do (vd theo canh gach),
//                go r lan nua -> so goc quaternion voi goc tich phan gyro
//           m  - doi che do NDOF (co la ban tu) <-> IMUPLUS (khong la ban)
//           c  - in trang thai hieu chuan + loi he thong
//
//  DOC KET QUA
//      - acc |g| ~ 9.8 m/s2 khi dung yen; temp 20-45 C.
//      - Hieu chuan S,G,A,M tu 0 den 3. Gyro (G) len 3 sau vai giay dung yen.
//      - Do troi: xe that do duoc ~4 do/phut (quaternion) trong khi gyro sau
//        khi tru bias ~0 - xem docs/SA_BAN_VA_TAI_LIEU.md.
//      - Quaternion toan 0 = BNO055 treo: TAT HAN NGUON XE roi bat lai (chi
//        bam reset ESP32 khong reset duoc BNO055).
//
//  Xong thi nap lai esp32_agv_base.ino.
// ============================================================================

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Giong esp32_agv_base.ino
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;
uint8_t bno_addr = 0x29;           // tu doi sang 0x28 neu quet thay 0x28

Adafruit_BNO055 *bno = nullptr;
adafruit_bno055_opmode_t mode = OPERATION_MODE_NDOF;

bool printing = true;
float yaw_zero = 0.0f;             // do, tu lenh z
uint32_t last_print = 0;

// Tich phan gyro z cho lenh r
bool rot_running = false;
double rot_gyro_deg = 0.0;
float rot_q_start = 0.0f;
uint32_t rot_last_us = 0;

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
  Serial.println("Quet I2C (SDA 21, SCL 22):");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  thay thiet bi o 0x%02X%s\n", a,
                    (a == 0x28 || a == 0x29) ? "  <- BNO055" : "");
      if (a == 0x28 || a == 0x29) bno_addr = a;
      n++;
    }
  }
  if (n == 0) Serial.println("  KHONG thay thiet bi nao - kiem tra 4 day: 3V3, GND, SDA->21, SCL->22.");
}

static void print_calib() {
  uint8_t s, g, a, m;
  bno->getCalibration(&s, &g, &a, &m);
  uint8_t st, test, err;
  bno->getSystemStatus(&st, &test, &err);
  Serial.printf("Hieu chuan S,G,A,M = %u,%u,%u,%u | trang thai %u, tu kiem tra 0x%02X (0x0F = dat), loi %u\n",
                s, g, a, m, st, test, err);
}

static void set_mode(adafruit_bno055_opmode_t m) {
  bno->setMode(m);
  mode = m;
  delay(30);
  Serial.printf("Che do: %s\n", m == OPERATION_MODE_NDOF ? "NDOF (co la ban tu - giong firmware)"
                                                       : "IMUPLUS (khong la ban tu)");
}

static void help() {
  Serial.println("Lenh: h huong dan | p bat/tat in | z dat 0 | d do troi 60 s | "
                 "r do quay tay | m doi NDOF/IMUPLUS | c hieu chuan");
}

// Do troi: dung yen 60 s, in moi 10 s
static void drift_test() {
  Serial.println("DO TROI 60 s - DE XE DUNG YEN, KHONG CHAM VAO XE...");
  delay(1000);
  float y0 = quat_yaw_deg();
  double gsum = 0.0;
  uint32_t n = 0, t0 = millis(), tnext = t0 + 10000;
  while (millis() - t0 < 60000) {
    gsum += bno->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE).z();
    n++;
    if (millis() >= tnext) {
      Serial.printf("  %2lu s: yaw troi %+6.2f do\n", (millis() - t0) / 1000,
                    wrap180(quat_yaw_deg() - y0));
      tnext += 10000;
    }
    delay(20);
  }
  float drift = wrap180(quat_yaw_deg() - y0);
  Serial.printf("KET QUA: quaternion troi %+.2f do/phut | gyro z trung binh (bias) %+.3f do/s (%lu mau)\n",
                drift, gsum / n, (unsigned long)n);
  Serial.println("  (|troi| < 1 do/phut la tot; firmware tu tru bias gyro do luc khoi dong)");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== TEST BNO055 ===");
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  i2c_scan();
  bno = new Adafruit_BNO055(55, bno_addr, &Wire);
  Serial.printf("Khoi tao BNO055 o 0x%02X ...\n", bno_addr);
  if (!bno->begin()) {
    Serial.println("KHONG khoi tao duoc BNO055. Kiem tra day, dia chi (chan ADR), nguon 3V3.");
    Serial.println("Neu truoc do chay binh thuong: TAT HAN NGUON XE vai giay roi bat lai.");
    while (true) delay(1000);
  }
  bno->setExtCrystalUse(true);
  delay(1000);
  set_mode(OPERATION_MODE_NDOF);
  print_calib();
  if (quat_all_zero())
    Serial.println("CANH BAO: quaternion toan 0 - BNO055 treo. TAT HAN NGUON XE roi bat lai.");
  yaw_zero = quat_yaw_deg();
  help();
}

void loop() {
  // Lenh tu Serial
  if (Serial.available()) {
    char ch = tolower(Serial.read());
    switch (ch) {
      case 'h': help(); break;
      case 'p': printing = !printing; Serial.printf("In lien tuc: %s\n", printing ? "BAT" : "TAT"); break;
      case 'z': yaw_zero = quat_yaw_deg(); Serial.println("Da dat yaw tuong doi = 0"); break;
      case 'd': drift_test(); break;
      case 'm': set_mode(mode == OPERATION_MODE_NDOF ? OPERATION_MODE_IMUPLUS : OPERATION_MODE_NDOF); break;
      case 'c': print_calib(); break;
      case 'r':
        if (!rot_running) {
          rot_running = true;
          rot_gyro_deg = 0.0;
          rot_q_start = quat_yaw_deg();
          rot_last_us = micros();
          Serial.println("DO QUAY: xoay xe bang tay (vd dung 90 do theo canh gach), xong go r lan nua.");
        } else {
          rot_running = false;
          float q_deg = wrap180(quat_yaw_deg() - rot_q_start);
          Serial.printf("KET QUA QUAY: quaternion %+.1f do | tich phan gyro %+.1f do | lech %.1f do\n",
                        q_deg, rot_gyro_deg, fabs(q_deg - rot_gyro_deg));
          Serial.println("  (xoay 90 do ma ca hai ~90 va lech < 3 do la tot)");
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

  // In 5 lan/giay
  if (printing && millis() - last_print >= 200) {
    last_print = millis();
    if (quat_all_zero()) {
      Serial.println("CANH BAO: quaternion toan 0 - BNO055 treo. TAT HAN NGUON XE roi bat lai.");
      return;
    }
    sensors_event_t e;
    bno->getEvent(&e);
    imu::Vector<3> g = bno->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> a = bno->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    float qy = quat_yaw_deg();
    uint8_t s, gc, ac, mc;
    bno->getCalibration(&s, &gc, &ac, &mc);
    Serial.printf("yaw %+7.2f (tuong doi %+7.2f) pitch %+6.1f roll %+6.1f | gyro z %+7.2f do/s | "
                  "acc |g| %5.2f | %2d C | cal %u%u%u%u%s\n",
                  qy, wrap180(qy - yaw_zero), e.orientation.y, e.orientation.z, g.z(),
                  sqrtf(a.x() * a.x() + a.y() * a.y() + a.z() * a.z()), bno->getTemp(),
                  s, gc, ac, mc, rot_running ? " | DANG DO QUAY" : "");
  }
}
