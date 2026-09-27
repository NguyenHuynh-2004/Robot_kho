// ========================================================
// ĐIỀU KHIỂN 1 ĐỘNG CƠ JGA25-370 + BTS7960 + PID
// Đọc encoder, chạy PWM cố định hoặc PID, điều khiển qua Serial.
// Serial Monitor: 115200 baud, chọn "Newline".
//
// Cần: Arduino-ESP32 core 3.x, thư viện ESP32Encoder
// Gõ ? để xem danh sách lệnh.
//
// Cập nhật:
// - In nguyên nhân khởi động lại (reset) mỗi lần ESP32 bật lên
// - Tăng/giảm PWM từ từ ở chế độ PWM cố định (lệnh o)
//   để tránh dòng khởi động lớn gây sụt áp
// ========================================================
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <esp_system.h>
#include <math.h>

// ========================================================
// 1. CHÂN KẾT NỐI (đổi ở đây nếu nối chân khác)
// ========================================================
const int RPWM_PIN  = 32;   // RPWM của BTS7960
const int LPWM_PIN  = 33;   // LPWM của BTS7960
const int ENC_A_PIN = 23;   // dây vàng encoder
const int ENC_B_PIN = 4;    // dây xanh lá encoder

// Đổi thành true nếu cần đảo chiều:
// INVERT_MOTOR  : PWM dương mà bánh quay lùi
// INVERT_ENCODER: bánh quay tiến mà RPM âm
const bool INVERT_MOTOR   = false;
const bool INVERT_ENCODER = false;

// ========================================================
// 2. THÔNG SỐ CỐ ĐỊNH
// ========================================================
const int PWM_FREQ = 5000;
const int PWM_RES = 8;
const int PWM_MAX = 255;

const float PULSES_PER_REV = 1200.0f;   // xung / vòng trục ra (HalfQuad)
const int PID_INTERVAL_MS = 20;         // chu kỳ PID 50 Hz
const int PRINT_INTERVAL_MS = 200;      // chu kỳ in Serial
const float MAX_COMMAND_RPM = 150.0f;

// Tốc độ thay đổi PWM ở chế độ PWM cố định:
// 5 PWM mỗi 20 ms -> từ 0 lên 255 mất khoảng 1 giây
const int OPEN_LOOP_PWM_STEP = 5;

// Bảo vệ sai chiều (encoder/dây động cơ bị đảo)
const int   WRONG_DIR_PWM = 120;
const float WRONG_DIR_RPM = 5.0f;
const unsigned long WRONG_DIR_TIME_MS = 300;

// Bảo vệ kẹt bánh / mất encoder
const int   STALL_PWM = 200;
const float STALL_RPM = 1.0f;
const unsigned long STALL_TIME_MS = 1000;

// ========================================================
// 3. THÔNG SỐ ĐIỀU KHIỂN (đổi được qua Serial)
// ========================================================
float maxAccelRPMPerSec = 320.0f;
float lpfAlpha = 0.2f;
float Kf = 1.47f;
int   deadbandPWM = 3;
float Kp = 0.3f;
float Ki = 0.02f;
float Kd = 0.01f;

// ========================================================
// 4. BIẾN TRẠNG THÁI
// ========================================================
ESP32Encoder encoder;

enum ControlMode {
  MODE_PID,
  MODE_OPEN_LOOP
};

ControlMode mode = MODE_PID;

float targetRPM = 0.0f;     // RPM đặt
float rampedRPM = 0.0f;     // RPM đặt sau ramp
float rawRPM = 0.0f;        // RPM đo thô
float currentRPM = 0.0f;    // RPM sau lọc
float previousRPM = 0.0f;
float integralTerm = 0.0f;

int outputPWM = 0;          // -255..255
int openLoopPWM = 0;        // PWM đích của lệnh o

long previousCount = 0;

bool emergencyStop = false;
bool plotMode = false;

unsigned long wrongDirSince = 0;
unsigned long stallSince = 0;

unsigned long nextPidTime = 0;
unsigned long lastPrintTime = 0;

// ========================================================
// 5. XUẤT PWM RA BTS7960
// ========================================================
void writeMotor(int pwm) {
  if (INVERT_MOTOR) {
    pwm = -pwm;
  }

  if (pwm > 0) {
    ledcWrite(RPWM_PIN, pwm);
    ledcWrite(LPWM_PIN, 0);
  }
  else if (pwm < 0) {
    ledcWrite(RPWM_PIN, 0);
    ledcWrite(LPWM_PIN, -pwm);
  }
  else {
    // Cả hai = 0 -> phanh
    ledcWrite(RPWM_PIN, 0);
    ledcWrite(LPWM_PIN, 0);
  }
}

// ========================================================
// 6. DỪNG KHẨN / DỪNG MỀM
// ========================================================
void triggerEmergencyStop(const char *reason) {
  emergencyStop = true;

  mode = MODE_PID;
  targetRPM = 0.0f;
  rampedRPM = 0.0f;
  integralTerm = 0.0f;
  outputPWM = 0;
  openLoopPWM = 0;
  wrongDirSince = 0;
  stallSince = 0;

  writeMotor(0);

  Serial.printf("\n!!! ESTOP: %s\n", reason);
  Serial.println("!!! Go 'g' de mo khoa.\n");
}

void softStop() {
  mode = MODE_PID;
  openLoopPWM = 0;
  targetRPM = 0.0f;
}

// ========================================================
// 7. BẢO VỆ
// ========================================================
void checkProtection(unsigned long now) {
  bool wrongDirection =
    abs(outputPWM) >= WRONG_DIR_PWM &&
    (
      (outputPWM > 0 && currentRPM < -WRONG_DIR_RPM) ||
      (outputPWM < 0 && currentRPM > WRONG_DIR_RPM)
    );

  if (wrongDirection) {
    if (wrongDirSince == 0) {
      wrongDirSince = now;
    }
    else if (now - wrongDirSince > WRONG_DIR_TIME_MS) {
      triggerEmergencyStop(
        "Banh quay nguoc chieu PWM. Kiem tra INVERT_MOTOR / INVERT_ENCODER.");
      return;
    }
  }
  else {
    wrongDirSince = 0;
  }

  bool stalled =
    abs(outputPWM) >= STALL_PWM &&
    fabsf(currentRPM) < STALL_RPM;

  if (stalled) {
    if (stallSince == 0) {
      stallSince = now;
    }
    else if (now - stallSince > STALL_TIME_MS) {
      triggerEmergencyStop("Banh ket hoac mat tin hieu encoder.");
    }
  }
  else {
    stallSince = 0;
  }
}

// ========================================================
// 8. MỘT CHU KỲ ĐIỀU KHIỂN (mỗi 20 ms)
// ========================================================
void controlStep() {
  const float dt = PID_INTERVAL_MS / 1000.0f;
  const float rampStep = maxAccelRPMPerSec * dt;

  // ---------- 1. Đọc encoder ----------
  long count = (long)encoder.getCount();
  long delta = count - previousCount;
  previousCount = count;

  if (INVERT_ENCODER) {
    delta = -delta;
  }

  rawRPM = ((float)delta / PULSES_PER_REV) * (60.0f / dt);

  // ---------- 2. Lọc thông thấp ----------
  currentRPM = lpfAlpha * rawRPM + (1.0f - lpfAlpha) * currentRPM;

  // ---------- 3. Tính PWM ----------
  if (emergencyStop) {
    targetRPM = 0.0f;
    rampedRPM = 0.0f;
    integralTerm = 0.0f;
    outputPWM = 0;
  }
  else if (mode == MODE_OPEN_LOOP) {
    // PWM cố định, không PID.
    // Tiến dần tới PWM đích để tránh dòng khởi động lớn.
    if (outputPWM < openLoopPWM) {
      outputPWM = min(outputPWM + OPEN_LOOP_PWM_STEP, openLoopPWM);
    }
    else if (outputPWM > openLoopPWM) {
      outputPWM = max(outputPWM - OPEN_LOOP_PWM_STEP, openLoopPWM);
    }

    integralTerm = 0.0f;
    rampedRPM = currentRPM;   // để chuyển sang PID không bị giật
  }
  else {
    // --- Ramp ---
    if (rampedRPM < targetRPM) {
      rampedRPM = fminf(rampedRPM + rampStep, targetRPM);
    }
    else if (rampedRPM > targetRPM) {
      rampedRPM = fmaxf(rampedRPM - rampStep, targetRPM);
    }

    if (fabsf(targetRPM) < 0.001f && fabsf(rampedRPM) < 0.001f) {
      integralTerm = 0.0f;
      outputPWM = 0;
    }
    else {
      float error = rampedRPM - currentRPM;

      // --- Tích phân + chống bão hòa ---
      bool saturatedHigh = outputPWM >= PWM_MAX && error > 0.0f;
      bool saturatedLow  = outputPWM <= -PWM_MAX && error < 0.0f;

      if (!saturatedHigh && !saturatedLow) {
        integralTerm += Ki * error;
      }

      integralTerm = constrain(integralTerm,
                               -(float)PWM_MAX, (float)PWM_MAX);

      // --- Vi phân theo giá trị đo ---
      float derivativeInput = currentRPM - previousRPM;

      // --- Feedforward ---
      float feedforward = 0.0f;

      if (rampedRPM > 0.5f) {
        feedforward = Kf * rampedRPM + deadbandPWM;
      }
      else if (rampedRPM < -0.5f) {
        feedforward = Kf * rampedRPM - deadbandPWM;
      }

      float output =
        feedforward +
        Kp * error +
        integralTerm -
        Kd * derivativeInput;

      outputPWM = constrain((int)output, -PWM_MAX, PWM_MAX);
    }
  }

  previousRPM = currentRPM;

  // ---------- 4. Bảo vệ ----------
  if (!emergencyStop) {
    checkProtection(millis());
  }

  // ---------- 5. Xuất PWM ----------
  writeMotor(outputPWM);
}

// ========================================================
// 9. IN TRẠNG THÁI
// ========================================================
void printStatus() {
  if (plotMode) {
    // Định dạng cho Serial Plotter
    Serial.printf("set:%.1f rpm:%.1f\n", rampedRPM, currentRPM);
    return;
  }

  Serial.printf(
    "%s%s set=%6.1f  ramp=%6.1f  rpm=%6.1f  pwm=%4d  count=%ld\n",
    emergencyStop ? "[ESTOP] " : "",
    mode == MODE_OPEN_LOOP ? "[PWM co dinh]" : "[PID]",
    mode == MODE_OPEN_LOOP ? 0.0f : targetRPM,
    rampedRPM,
    currentRPM,
    outputPWM,
    (long)encoder.getCount()
  );
}

void printParameters() {
  Serial.println("---------- THONG SO ----------");
  Serial.printf("acc   = %.1f RPM/s\n", maxAccelRPMPerSec);
  Serial.printf("alpha = %.3f\n", lpfAlpha);
  Serial.printf("kf    = %.3f\n", Kf);
  Serial.printf("db    = %d\n", deadbandPWM);
  Serial.printf("kp    = %.4f\n", Kp);
  Serial.printf("ki    = %.4f\n", Ki);
  Serial.printf("kd    = %.4f\n", Kd);
  Serial.println("------------------------------");
}

void printHelp() {
  Serial.println();
  Serial.println("=========== DANH SACH LENH ===========");
  Serial.println("r <rpm>      : chay PID              (vd: r 20, r -20)");
  Serial.println("o <pwm>      : chay PWM co dinh      (vd: o 60, o -60)");
  Serial.println("s            : dung mem");
  Serial.println("e            : DUNG KHAN");
  Serial.println("g            : mo khoa dung khan");
  Serial.println("kp/ki/kd/kf <gia tri> : doi he so   (vd: kp 0.4)");
  Serial.println("db <pwm>     : deadband PWM");
  Serial.println("acc <rpm/s>  : gia toc toi da");
  Serial.println("alpha <0..1> : he so loc LPF");
  Serial.println("z            : xoa bo dem encoder");
  Serial.println("plot         : bat/tat che do Serial Plotter");
  Serial.println("?            : xem lenh va thong so");
  Serial.println("======================================");
  Serial.println();
}

// ========================================================
// 10. XỬ LÝ LỆNH SERIAL
// ========================================================
bool parseNumber(const char *text, float &result) {
  char *end = nullptr;
  result = strtof(text, &end);
  return end != text && *end == '\0' && isfinite(result);
}

bool setInRange(float &destination, float value, float low, float high) {
  if (value < low || value > high) {
    return false;
  }
  destination = value;
  return true;
}

void processCommand(char *line) {
  char *token[3];
  int count = 0;

  char *part = strtok(line, " ,\t");
  while (part != nullptr && count < 3) {
    token[count++] = part;
    part = strtok(nullptr, " ,\t");
  }

  if (count == 0) {
    return;
  }

  for (char *c = token[0]; *c; c++) {
    *c = tolower(*c);
  }

  const char *cmd = token[0];

  // ---------- Lệnh không tham số ----------
  if (!strcmp(cmd, "?") || !strcmp(cmd, "help")) {
    printHelp();
    printParameters();
    return;
  }

  if (!strcmp(cmd, "e")) {
    triggerEmergencyStop("Lenh tu Serial");
    return;
  }

  if (!strcmp(cmd, "g")) {
    emergencyStop = false;
    Serial.println("Da mo khoa ESTOP. RPM hien = 0.");
    return;
  }

  if (!strcmp(cmd, "s")) {
    softStop();
    Serial.println("Dung mem.");
    return;
  }

  if (!strcmp(cmd, "z")) {
    encoder.clearCount();
    previousCount = 0;
    Serial.println("Da xoa bo dem encoder.");
    return;
  }

  if (!strcmp(cmd, "plot")) {
    plotMode = !plotMode;
    Serial.println(plotMode ? "Che do Plotter: BAT" : "Che do Plotter: TAT");
    return;
  }

  // ---------- Lệnh có 1 tham số ----------
  float value;

  if (count != 2 || !parseNumber(token[1], value)) {
    Serial.println("Lenh khong hop le. Go ? de xem huong dan.");
    return;
  }

  if (!strcmp(cmd, "r")) {
    if (emergencyStop) {
      Serial.println("ESTOP dang bat. Go 'g' de mo khoa truoc.");
      return;
    }
    if (fabsf(value) > MAX_COMMAND_RPM) {
      Serial.printf("RPM phai trong khoang +-%.0f\n", MAX_COMMAND_RPM);
      return;
    }
    mode = MODE_PID;
    openLoopPWM = 0;
    targetRPM = value;
    Serial.printf("PID -> %.1f RPM\n", value);
    return;
  }

  if (!strcmp(cmd, "o")) {
    if (emergencyStop) {
      Serial.println("ESTOP dang bat. Go 'g' de mo khoa truoc.");
      return;
    }
    if (fabsf(value) > PWM_MAX) {
      Serial.println("PWM phai trong khoang -255..255");
      return;
    }
    mode = MODE_OPEN_LOOP;
    openLoopPWM = (int)value;
    Serial.printf("PWM co dinh -> %d\n", openLoopPWM);
    return;
  }

  bool ok;

  if      (!strcmp(cmd, "kp"))    ok = setInRange(Kp, value, 0.0f, 20.0f);
  else if (!strcmp(cmd, "ki"))    ok = setInRange(Ki, value, 0.0f, 20.0f);
  else if (!strcmp(cmd, "kd"))    ok = setInRange(Kd, value, 0.0f, 20.0f);
  else if (!strcmp(cmd, "kf"))    ok = setInRange(Kf, value, 0.0f, 10.0f);
  else if (!strcmp(cmd, "acc"))   ok = setInRange(maxAccelRPMPerSec, value, 1.0f, 3000.0f);
  else if (!strcmp(cmd, "alpha")) ok = setInRange(lpfAlpha, value, 0.001f, 1.0f);
  else if (!strcmp(cmd, "db")) {
    ok = value >= 0.0f && value <= 255.0f;
    if (ok) {
      deadbandPWM = (int)lroundf(value);
    }
  }
  else {
    Serial.println("Lenh khong hop le. Go ? de xem huong dan.");
    return;
  }

  if (ok) {
    printParameters();
  }
  else {
    Serial.println("Gia tri ngoai khoang cho phep.");
  }
}

char lineBuffer[64];
int lineLength = 0;

void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (lineLength > 0) {
        lineBuffer[lineLength] = '\0';
        processCommand(lineBuffer);
        lineLength = 0;
      }
    }
    else if (lineLength < (int)sizeof(lineBuffer) - 1) {
      lineBuffer[lineLength++] = c;
    }
  }
}

// ========================================================
// 11. IN NGUYÊN NHÂN KHỞI ĐỘNG LẠI
// ========================================================
void printResetReason() {
  esp_reset_reason_t reason = esp_reset_reason();

  Serial.println();
  Serial.println("======================================");

  switch (reason) {
    case ESP_RST_POWERON:
      Serial.println(">>> Reset: CAP NGUON (bat nguon / mat nguon hoan toan)");
      break;

    case ESP_RST_BROWNOUT:
      Serial.println(">>> Reset: SUT AP (brownout)");
      Serial.println("    -> Kiem tra cap USB, them tu 470-1000uF o 5V/GND ESP32");
      break;

    case ESP_RST_PANIC:
      Serial.println(">>> Reset: LOI CHUONG TRINH (panic)");
      Serial.println("    -> Chep doan 'Guru Meditation' / 'Backtrace' phia tren");
      break;

    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
      Serial.println(">>> Reset: WATCHDOG (chuong trinh bi treo)");
      break;

    case ESP_RST_SW:
      Serial.println(">>> Reset: PHAN MEM (goi esp_restart)");
      break;

    case ESP_RST_EXT:
      Serial.println(">>> Reset: NUT EN / CHAN RESET");
      break;

    default:
      Serial.printf(">>> Reset: ma %d\n", (int)reason);
      break;
  }

  Serial.println("======================================");
}

// ========================================================
// 12. SETUP
// ========================================================
void setup() {
  // PWM về 0 càng sớm càng tốt
  ledcAttach(RPWM_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(LPWM_PIN, PWM_FREQ, PWM_RES);
  writeMotor(0);

  Serial.begin(115200);
  delay(300);

  printResetReason();

  encoder.attachHalfQuad(ENC_A_PIN, ENC_B_PIN);
  encoder.setFilter(1023);
  encoder.clearCount();
  previousCount = (long)encoder.getCount();

  nextPidTime = millis() + PID_INTERVAL_MS;
  lastPrintTime = millis();

  printHelp();
  printParameters();
}

// ========================================================
// 13. LOOP
// ========================================================
void loop() {
  handleSerial();

  unsigned long now = millis();

  if ((long)(now - nextPidTime) >= 0) {
    nextPidTime += PID_INTERVAL_MS;

    if ((long)(now - nextPidTime) > 5 * PID_INTERVAL_MS) {
      nextPidTime = now + PID_INTERVAL_MS;
    }

    controlStep();
  }

  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;
    printStatus();
  }
}
