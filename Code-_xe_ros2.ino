#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <esp_wifi.h>
#include <rmw_microros/rmw_microros.h> // THÊM THƯ VIỆN NÀY ĐỂ SỬA LỖI CLIENT KEY

// -------- Thiết lập Định danh Xe (Sửa trước khi nạp code) --------
// Thay đổi thành 2 cho xe thứ hai
#define ROBOT_ID 2

// ==========================================
// THÊM THƯ VIỆN CẢM BIẾN BNO055
// ==========================================
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);

// Biến lưu trữ góc của xe
float current_yaw = 0.0;
float target_yaw = 0.0;

// ==========================================
// CÁC BIẾN CẤU HÌNH & TRẠNG THÁI
// ==========================================

#define AIN1 23
#define AIN2 17
#define BIN1 16
#define BIN2 19
#define PWMA 5
#define PWMB 15

const int NUM_SENSORS = 5;
int sensorGPIO[NUM_SENSORS] = {34, 35, 32, 33, 36}; 

bool isBlackLine = 1;  
int minValues[NUM_SENSORS] = {0, 0, 0, 0, 0};       
int maxValues[NUM_SENSORS] = {3300, 3300, 3300, 3300, 3300}; 

rcl_subscription_t subscriber;
std_msgs__msg__String msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.println("microROS Error!");}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

enum RunState { FOLLOW, MANEUVER, HALT };

// Từ khóa "volatile" để 2 lõi (Core 0 và Core 1) đọc/ghi biến an toàn
volatile RunState state = HALT; 
volatile char maneuver_cmd = 'F';
volatile unsigned long maneuver_t_start = 0;

float P = 0, I = 0, D = 0, previousError = 0, PIDvalue = 0;
float error = 0;
int   lsp = 0, rsp = 0;

// ==========================================
// THÔNG SỐ TỐC ĐỘ BÁM LINE
// ==========================================
int   lfSpeed = 90;       
int   currentSpeed = 60;   

float sensorWeight[NUM_SENSORS] = { 4, 1, 0, -1, -4 };
float Kp = 15.0f; 
float Kd = 80.0f; 
float Ki = 0.00f; 

int onLine = 1;
int sensorValue[NUM_SENSORS];
int sensorArray[NUM_SENSORS];
int sumValue = 0;                  

// ==========================================
// THÔNG SỐ TỐC ĐỘ NGÃ TƯ
// ==========================================
const int STRAIGHT_SPEED = 90;
const int BACK_SPEED     = -90;
const int TURN_TIMEOUT   = 3000; // Timeout an toàn 3s
const int STRAIGHT_MS    = 420;

// Các hàm khai báo nhanh
inline void tank(int left, int right);

// ==========================================
// CÁC HÀM ĐIỀU KHIỂN RẼ BẰNG BNO055
// ==========================================
void startManeuver(char c) {
  maneuver_cmd = c;
  maneuver_t_start = millis();
  
  // Đọc góc hiện tại để làm mốc tính toán
  sensors_event_t event;
  bno.getEvent(&event);
  current_yaw = event.orientation.x;

  if (c == 'R') {
    target_yaw = current_yaw + 90.0;
    if (target_yaw >= 360.0) target_yaw -= 360.0;
  } else if (c == 'L') {
    target_yaw = current_yaw - 90.0;
    if (target_yaw < 0.0) target_yaw += 360.0;
  } else if (c == 'B') {
    target_yaw = current_yaw + 180.0;
    if (target_yaw >= 360.0) target_yaw -= 360.0;
  }

  state = MANEUVER;
  Serial.print(">> Maneuver: "); Serial.println(c);
}

bool stepManeuver() {
  unsigned long t = millis() - maneuver_t_start;

  // Đi thẳng vẫn dùng theo mốc thời gian để chạy qua tâm ngã tư
  if (maneuver_cmd == 'F') {
    tank(STRAIGHT_SPEED, STRAIGHT_SPEED);
    if (t > STRAIGHT_MS) { tank(0,0); return true; }
    return false;
  }

  // =======================================================
  // ĐÃ SỬA: Nhích xe lên trước 0.8s (800ms) với tốc độ 70
  // =======================================================
  if (t < 800) {
    tank(90, 90);
    return false; 
  }

  // --- LOGIC RẼ (R, L, B) DÙNG LA BÀN BNO055 ---
  sensors_event_t event;
  bno.getEvent(&event);
  current_yaw = event.orientation.x;

  float err = target_yaw - current_yaw;
  
  // Xử lý góc xoay vòng (Đảm bảo xe quay đường ngắn nhất)
  if (err > 180.0) err -= 360.0;
  if (err < -180.0) err += 360.0;

  // Đạt góc đích (sai số dưới 2 độ) -> Xong lệnh
  if (abs(err) <= 2.0) {
    tank(0,0);
    return true; 
  }

  // Thuật toán P đơn giản cho tốc độ quay
  float turn_kp = 2.0; 
  int turn_effort = abs(err * turn_kp);

  // ==========================================
  // THÔNG SỐ GIỚI HẠN VẬN TỐC RẼ
  // ==========================================
  int min_turn = 70;  // Vận tốc tối thiểu
  int max_turn = 80;  // Giới hạn trần lớn nhất
  
  if (turn_effort < min_turn) turn_effort = min_turn;
  if (turn_effort > max_turn) turn_effort = max_turn;

  if (err > 0) {
    tank(turn_effort, -turn_effort); // Quay Phải
  } else {
    tank(-turn_effort, turn_effort); // Quay Trái
  }

  // Chống kẹt bánh (Cộng thêm 800ms vì đã mất 0.8s nhích tới)
  if (t > TURN_TIMEOUT + 800) { 
    tank(0,0); 
    return true; 
  }
  
  return false;
}

// ==========================================
// NHẬN LỆNH TỪ ROS (Chạy trên Core 0)
// ==========================================
void subscription_callback(const void * msgin) {
  const std_msgs__msg__String * new_msg = (const std_msgs__msg__String *)msgin;
  if (new_msg->data.size > 0) {
    char cmd = toupper(new_msg->data.data[0]);
    Serial.print(">> Nhan lenh ROS: "); Serial.println(cmd);

    if (cmd == 'L' || cmd == 'R' || cmd == 'F' || cmd == 'B') {
      startManeuver(cmd);
    } else if (cmd == 'S') {
      state = HALT;      
      tank(0, 0);
      Serial.println(">> Phanh xe!");
    }
  }
}

// Nhiệm vụ (Task) độc lập chuyên lắng nghe ROS trên Core 0
void ros_task(void * pvParameters) {
  for (;;) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
    vTaskDelay(1 / portTICK_PERIOD_MS); 
  }
}

// ==========================================
// ĐIỀU KHIỂN ĐỘNG CƠ
// ==========================================
void motor1run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); analogWrite(PWMA, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); analogWrite(PWMA, -motorSpeed);
  } else {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH); analogWrite(PWMA, 0);
  }
}

void motor2run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); analogWrite(PWMB, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); analogWrite(PWMB, -motorSpeed);
  } else {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH); analogWrite(PWMB, 0);
  }
}

inline void tank(int left, int right) { motor1run(left); motor2run(right); }

// ==========================================
// ĐỌC CẢM BIẾN LINE
// ==========================================
void readLineAnalog() {
  onLine = 0; sumValue = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = analogRead(sensorGPIO[i]); 
    int v;
    
    if (isBlackLine) v = map(raw, minValues[i], maxValues[i], 1000, 0);
    else             v = map(raw, minValues[i], maxValues[i], 0, 1000);
    
    v = constrain(v, 0, 1000);
    sensorValue[i] = v;
    
    sensorArray[i] = (v > 500) ? 1 : 0;
    if (sensorArray[i]) onLine = 1;
    sumValue += v;
  }
}

// ==========================================
// PID BÁM LINE
// ==========================================
void linefollow() {
  float numerator = 0, denom = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    numerator += sensorWeight[i] * (float)sensorValue[i];
    denom     += (float)sensorValue[i];
  }
  if (denom >= 200.0f) error = numerator / denom;
  
  P = error;
  I += error; I = constrain(I, -1000.0f, 1000.0f);
  D = error - previousError;
  PIDvalue = Kp * P + Ki * I + Kd * D;
  previousError = error;

  lsp = constrain((int)(currentSpeed - PIDvalue), 0, 255);
  rsp = constrain((int)(currentSpeed + PIDvalue), 0, 255);
  tank(lsp, rsp);
}

// ==========================================
// CẤU HÌNH HỆ THỐNG
// ==========================================
void setup() {
  Serial.begin(115200);

  // --- 1. KHỞI TẠO BNO055 ---
  if (!bno.begin()) {
    Serial.println("LỖI: Khong tim thay BNO055. Kiem tra day noi I2C!");
    while (1); 
  }
  bno.setExtCrystalUse(true);
  Serial.println("BNO055 san sang!");
  delay(500);

  // --- 2. KHỞI TẠO WI-FI & MICRO-ROS ---
  set_microros_wifi_transports("SV BACH KHOA", "YOUR_WIFI_PASSWORD", "192.168.1.21", 8888);
  esp_wifi_set_ps(WIFI_PS_NONE);

  Serial.print("Dang ket noi Wi-Fi...");
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(500);
    Serial.print(".");
    retries++;
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("LỖI: Khong the ket noi Wi-Fi!");
    while(1); 
  }
  
  Serial.print("Wi-Fi OK! IP cua xe: ");
  Serial.println(WiFi.localIP());
  
  // Tạo tên node và tên topic riêng biệt cho từng xe
  char node_name[30];
  sprintf(node_name, "esp32_agv_node_%d", ROBOT_ID);

  char topic_name[20];
  sprintf(topic_name, "agv%d_command", ROBOT_ID);

  // ĐẶT CLIENT KEY DUY NHẤT ĐỂ TRÁNH XUNG ĐỘT (LỖI 0x7D248957)
  allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  
  rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
  rmw_uros_options_set_client_key(0x11111111 * ROBOT_ID, rmw_options);

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, node_name, "", &support));
  
  RCCHECK(rclc_subscription_init_best_effort(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    topic_name));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  
  msg.data.data = (char *) malloc(10 * sizeof(char)); 
  msg.data.size = 0;
  msg.data.capacity = 10;

  // --- 3. CẤU HÌNH GPIO ---
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(PWMB, OUTPUT);
  for (int i = 0; i < NUM_SENSORS; i++) pinMode(sensorGPIO[i], INPUT);

  // --- 4. TẠO TASK TRÊN CORE 0 ---
  xTaskCreatePinnedToCore(
    ros_task,     // Tên hàm chạy
    "ROS_Task",   // Tên định danh
    10000,        // Dung lượng RAM cấp phát
    NULL,         // Tham số 
    1,            // Độ ưu tiên
    NULL,         // Quản lý task
    0             // Chạy trên Core 0
  );

  Serial.println("Hệ thống Dual-Core sẵn sàng.");
}

// ==========================================
// VÒNG LẶP CHÍNH (Chạy trên Core 1 - Chỉ xử lý PID & Rẽ)
// ==========================================
void loop() {
  readLineAnalog();

  if (state == HALT) {
    tank(0,0);
    lsp = 0; rsp = 0; 
    currentSpeed = 60; 
  } 
  else {
    // Gia tốc mềm (Soft Start) khi bắt đầu chạy
    static unsigned long lastAccel = 0;
    if (millis() - lastAccel > 2) { 
      lastAccel = millis();
      if (currentSpeed < lfSpeed) currentSpeed++;
    }

    if (state == FOLLOW) {
      if (onLine == 1) {
        linefollow();
      } else {
        // Mất line: giữ lại tốc độ rẽ dựa trên sai số cuối cùng
        if (error > 0) { tank(0, lfSpeed); lsp = 0; rsp = lfSpeed; }
        else           { tank(lfSpeed, 0); lsp = lfSpeed; rsp = 0; }
      }
    } 
    else if (state == MANEUVER) {
      // Gọi hàm bước rẽ, nếu rẽ xong (trả về true) thì quay lại bám line
      if (stepManeuver()) {
        state = FOLLOW; 
        I = 0; // Reset thông số PID chống văng đuôi xe
      }
    }
  }

  // ==========================================
  // IN DỮ LIỆU RA SERIAL MONITOR 
  // ==========================================
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 100) { 
    lastPrintTime = millis();
    
    Serial.print("CB: [");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(sensorValue[i]);
      if (i < NUM_SENSORS - 1) Serial.print(",");
    }
    Serial.print("] | ");
    
    if (state == MANEUVER) {
      Serial.print("ĐANG RẼ (Góc IMU: "); 
      Serial.print(current_yaw);
      Serial.print(" -> Đích: ");
      Serial.print(target_yaw);
      Serial.println(")");
    } else if (state == HALT) {
      Serial.println("ĐANG DỪNG (HALT)");
    } else {
      Serial.print("LineErr: ");
      Serial.print(error);
      Serial.print(" | Động cơ: ");
      Serial.print(lsp);
      Serial.print("/");
      Serial.println(rsp);
    }
  }
}