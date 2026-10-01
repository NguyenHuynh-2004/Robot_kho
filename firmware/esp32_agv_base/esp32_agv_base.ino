// ============================================================================
//  XE AGV THAT - ESP32 DON, micro-ROS qua Wi-Fi/UDP
// ============================================================================
//  Thay the hoan toan plugin DiffDrive + gpu_lidar cua Gazebo, de Nav2 chay
//  tren xe that y het nhu trong mo phong.
//
//  NHAN (subscribe)
//      /cmd_vel        geometry_msgs/Twist     <- Nav2
//  PHAT (publish)
//      /scan           sensor_msgs/LaserScan   <- YDLIDAR X2/X2L
//      /odom           nav_msgs/Odometry       <- encoder + gyro
//      /tf             tf2_msgs/TFMessage      <- odom -> base_footprint
//      /imu/data       sensor_msgs/Imu         <- BNO055
//
//  Ghep tu 3 sketch goc:
//      ydlidar_microros.ino  -> phan LiDAR (gan nhu nguyen ven)
//      motor_1_wheel.ino     -> PID + feedforward + bao ve, nhan doi cho 2 banh
//      Code-_xe_ros2.ino     -> khung micro-ROS Wi-Fi + BNO055
//  Da bo: bam line, 5 cam bien quang, lenh L/R/F/B, std_msgs/String,
//         giao dien lenh Serial.
//
//  PHAN CHIA LOI
//      Core 1  loop()       : CHI chay lidar.loop() - tuyet doi khong block
//      Core 0  motor_task   : 50 Hz, uu tien 3 - PID 2 banh + tich phan odom
//      Core 0  ros_task     : uu tien 1 - executor + publish + doc BNO055
//
//  Laptop: ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
//
//  THIET LAP BOARD (Arduino IDE -> Tools)
//      Board            : ESP32 Dev Module   (esp32:esp32:esp32)
//      Partition Scheme : Huge APP (3MB No OTA/1MB SPIFFS)   <-- QUAN TRONG
//
//  Da bien dich thu bang arduino-cli 1.5.2 voi core esp32 3.3.8:
//      Partition mac dinh : 1072922 byte = 81% cua 1310720  (sat tran)
//      Huge APP           : 1072954 byte = 34% cua 3145728  (nen dung)
//  RAM ca hai truong hop: 122744 byte = 37%.
//  De partition mac dinh thi chi can them vai tinh nang nua la tran flash.
//
//  Thu vien can co (deu da co san tren may nay):
//      micro_ros_arduino 2.0.7-iron, LDS 0.6.3, ESP32Encoder 5.0.0,
//      Adafruit BNO055 1.6.4, Adafruit BusIO 1.17.4, Adafruit Unified Sensor 1.1.15
// ============================================================================

#ifndef ESP32
  #error Sketch nay chi chay tren ESP32
#endif

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <math.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <ESP32Encoder.h>
#include <LDS_YDLIDAR_X2_X2L.h>

#include <esp_timer.h>   // esp_timer_get_time() - dong ho us 64 bit, lam timestamp

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <tf2_msgs/msg/tf_message.h>
// tf_message.h chi keo struct cua TransformStamped, khong keo cac ham __init.
// Thieu dong duoi la loi bien dich: '...TransformStamped__init' was not declared.
#include <geometry_msgs/msg/transform_stamped.h>

// ============================================================================
//  1. CAU HINH - SUA TRUOC KHI NAP
// ============================================================================

// --- Mang ---
#define WIFI_SSID   "SV BACH KHOA"
#define WIFI_PASS   "YOUR_WIFI_PASSWORD"
#define AGENT_IP    "192.168.1.21"     // IP laptop dang chay micro_ros_agent
#define AGENT_PORT  8888
#define ROBOT_ID    1                  // doi neu chay nhieu xe cung luc

// --- Ten frame: PHAI khop robot.urdf.xacro ---
#define ODOM_FRAME  "odom"
#define BASE_FRAME  "base_footprint"
#define LASER_FRAME "lidar_link"
#define IMU_FRAME   "imu_link"

// --- KICH THUOC XE: DO BANG THUOC, DUNG DOAN ---------------------------------
// Sai o hai so nay la odometry sai, va Nav2 se khong bao gio chay dung.
// Quy trinh hieu chinh nam o cuoi file.
// Do bang thuoc tren xe that. PHAI khop robot.urdf.xacro va gazebo_control.xacro.
#define WHEEL_RADIUS_M      0.03225f   // duong kinh banh sau 64.5 mm
#define WHEEL_SEPARATION_M  0.200f     // khoang cach TAM - TAM hai banh sau
// WHEEL_RADIUS_M o tren la ban kinh HINH HOC. Ban kinh LAN thuc te nho hon vi
// lop bi nen duoi tai -> xe se di ngan hon lenh. Hieu chinh theo Phu luc A3.
// -----------------------------------------------------------------------------

// Xung tren mot vong TRUC RA, voi attachHalfQuad (dem 2 canh).
//
// DO DUOC bang firmware/encoder_test/encoder_test.ino, quay tay banh:
//      10 vong : TRAI 21614 -> 2161.4      PHAI 21375 -> 2137.5
//       3 vong : TRAI  6423 -> 2141.0      PHAI  6401 -> 2133.7
// Lay trung binh phep 10 vong -> 2150.
//
// Khop voi phan cung: encoder 11 PPR * 2 (halfQuad) = 22 xung moi vong truc
// MOTOR, nhan ti so hop so -> 2150/22 = 97.7. Hop so hanh tinh co nho hiem khi
// ra ti so nguyen nen 1:97.7 la binh thuong; dung ep ve 2200 (=1:100) cho tron.
//
// So cu la 1200, chep tu sketch goc va chua ai kiem - sai 78%. Do chinh la
// nguyen nhan odometry bao thua: do chay thang 2.00 m ra 3.494 m, dung bang
// 2.000 * 2150/1200 = 3.58 m trong vong 2%.
//
// Do lai bang encoder_test.ino neu doi dong co. Sai so con lai sau khi nap thi
// bu bang tham so wheel_radius tren laptop, khong phai nap chip lan nua.
#define ENC_PULSES_PER_REV  2150.0f

// Gioi han an toan
#define MAX_WHEEL_RPM       120.0f     // tran RPM moi banh (L298N sut ap -> thap hon BTS7960)
#define CMD_VEL_TIMEOUT_MS  500        // mat /cmd_vel qua lau -> dung xe

// ============================================================================
//  2. CHAN KET NOI
// ============================================================================
// Da chon lai de 3 sketch goc khong dam nhau (ban goc: LiDAR va motor cung
// dung 32/33). Tranh GPIO 0/2/5/12/15 (chan strapping - muc logic luc khoi
// dong quyet dinh che do boot) va 6..11 (noi voi flash).

// --- TB6612FNG / L298N ---
// L298N:  IN1  IN2  ENA   ket qua
//         HIGH LOW  PWM   quay thuan
//         LOW  HIGH PWM   quay nghich
//         LOW  LOW  0     tha troi
// PHAI THAO jumper ENA/ENB tren module thi chan PWM moi co tac dung.
const int L_IN1_PIN = 27;   // banh TRAI
const int L_IN2_PIN = 26;
const int L_ENA_PIN = 25;   // PWM banh trai
const int R_IN1_PIN = 33;   // banh PHAI
const int R_IN2_PIN = 32;
const int R_ENB_PIN = 14;   // PWM banh phai

// --- Encoder ---
// GPIO 34/35 chi vao (input-only) va KHONG co dien tro keo len noi bo.
// Encoder Hall kieu push-pull chay tot; neu encoder cua ban la ho cuc thu
// (open-collector) thi phai han them dien tro keo len 10k ra 3V3.
const int ENC_L_A_PIN = 23;
const int ENC_L_B_PIN = 34;
const int ENC_R_A_PIN = 4;
const int ENC_R_B_PIN = 35;

// --- BNO055 (I2C) ---
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;
#define BNO055_ADDR 0x29           // 0x28 neu chan ADR noi GND

// --- YDLIDAR X2/X2L (loai 4 chan: VCC, GND, Tx, PWM) ---
// LiDAR nay chi PHAT du lieu, khong nhan lenh: LDS_YDLIDAR_X2_X2L::start()
// chi goi enableMotor(true) va khong ghi byte nao ra UART. Vi vay khong can
// chan TX cua ESP32, cung khong can chan EN.
//   -1 = khong cap phat chan nao (HardwareSerial::begin va callback deu hieu).
const int8_t LIDAR_RX_PIN  = 16;   // ESP32 RX <- Tx cua LiDAR   (BAT BUOC)
const int8_t LIDAR_PWM_PIN = 18;   // -> chan PWM cua LiDAR      (BAT BUOC)
const int8_t LIDAR_TX_PIN  = -1;   // LiDAR khong co chan Rx     -> bo trong
const int8_t LIDAR_EN_PIN  = -1;   // driver X2/X2L khong dung   -> bo trong

// --- Dao chieu: sua neu lap nguoc ---
// INVERT_MOTOR_x  : cap PWM duong ma banh chay lui
// INVERT_ENC_x    : banh chay toi ma so xung giam
//
// Bo gia tri duoi day DO DUOC tren xe that, khong phai doan:
//   /joy axes[1] = +1.00  ->  /cmd_vel linear.x = +0.250  ->  odom = +0.260
// tuc lenh dung la TIEN, nhung xe chay LUI. Ca hai dong co lap nguoc, va
// odometry khi do noi doi: bao tien trong khi xe lui.
//
// PHAI DAO CA BON CO CUNG LUC. Chi dao dong co thoi thi banh se quay nguoc
// chieu PWM, wheel_check_protection phat hien sau 300 ms va kich ESTOP.
// Ly do: bo cu von nhat quan (PID khong keu gi), nghia la xung encoder thô
// dang khop voi chieu quay cu; lat chieu quay thi xung thô doi dau theo, nen
// co encoder cung phai lat.
//
// Neu sau nay ban dao lai day o dau ra L298N thi nho tra bo nay ve nhu cu.
const bool INVERT_MOTOR_L = true;
const bool INVERT_MOTOR_R = false;
const bool INVERT_ENC_L   = true;
const bool INVERT_ENC_R   = false;

// ============================================================================
//  3. THONG SO DIEU KHIEN
// ============================================================================
#define PWM_FREQ 5000                // L298N dung BJT cham, khong chay noi 20 kHz
#define PWM_RES  8
#define PWM_MAX  255

#define PID_INTERVAL_MS 20           // 50 Hz, giong motor_1_wheel.ino

// He so lay tu motor_1_wheel_l298n.ino (ban L298N, KHONG phai ban BTS7960).
// L298N sut ap ~2V qua cau H nen deadband lon hon han: 30 thay vi 3.
// Van nen do lai tren xe cua ban, xem Phu luc B.
float Kf    = 1.8f;                  // feedforward: PWM tren moi RPM
float Kp    = 0.30f;
float Ki    = 0.02f;
float Kd    = 0.01f;
int   deadbandPWM = 30;
float lpfAlpha    = 0.2f;            // loc thong thap cho RPM do duoc
float maxAccelRPMPerSec = 320.0f;    // ramp, chong giat va sut ap

// Bao ve (giu nguyen tu motor_1_wheel.ino)
#define WRONG_DIR_PWM     120
#define WRONG_DIR_RPM     5.0f
#define WRONG_DIR_TIME_MS 300
#define STALL_PWM         200
#define STALL_RPM         1.0f
#define STALL_TIME_MS     1000

// ----------------------------------------------------------------------------
// AI TINH ODOMETRY?
//   0 = LAPTOP tinh  (mac dinh). ESP32 chi phat /joint_states (goc banh TICH
//       LUY, radian) va /imu/data. Node wheel_odometry.py tren laptop ghep lai
//       thanh /odom va TF odom->base_footprint.
//   1 = ESP32 tu tinh va phat /odom + /tf nhu truoc. Giu lam duong lui.
//
// VI SAO CHUYEN LEN LAPTOP:
//   - Sua mot y tuong ve odometry khong con phai nap lai chip.
//   - WHEEL_RADIUS_M roi khoi duong tinh odometry: ESP32 gui GOC banh, laptop
//     nhan voi ban kinh. Hieu chinh ban kinh lan = sua mot tham so ROS.
//   - Laptop tinh bang float64 va co the dung quaternion tuyet doi cua BNO055
//     thay vi tich phan toc do goc (tich phan thi troi, quaternion thi khong).
//
// DIEU KIEN DE LAM DUOC: phai gui gia tri TICH LUY, khong gui so gia.
//   Da do duoc tren duong truyen nay: /odom co duoi tre toi 32 ms, /scan mat 7
//   khoang > 200 ms trong 12 giay. Neu gui "chu ky vua roi banh di 3 mm" thi
//   moi goi roi la mat han 3 mm, sai so khong bao gio lay lai duoc. Gui "goc
//   banh hien tai la 1234.56 rad" thi goi roi khong ton gi.
// ----------------------------------------------------------------------------
#define ODOM_ON_ESP32 0

// Chi con y nghia khi ODOM_ON_ESP32 = 1.
//   1 = tich phan gyro Z cua BNO055  (chinh xac hon HAN, nen dung)
//   0 = suy tu chenh lech 2 encoder  (du phong khi IMU hong)
#define USE_GYRO_FOR_YAW 1

// ============================================================================
//  4. CAU HINH LiDAR  (giu nguyen tu ydlidar_microros.ino)
// ============================================================================
// Huong quay: YDLIDAR quay CUNG chieu kim dong ho, ROS quy uoc goc tang NGUOC
// chieu kim dong ho -> mac dinh lat dau. Anh quet bi lat guong thi doi thanh 0.
#define LIDAR_ANGLE_FLIP       1
// Bu lech goc khi gan LiDAR len xe.
//
// DA DO, khong con la so dat tam. Dat mot vat rong ~36 cm ngay truoc mui xe,
// cach 1.20 m, roi chay tools/check_lidar_offset.py (trung vi truot +-2 bin,
// 25 vong quet): vat hien ra o phuong vi 90.0 do thay vi 0 do. Tuc voi
// OFFSET = 90 thi anh quet bi xoay du dung mot goc vuong -> doi thanh 180.
//
// Cach suy: bearing phat ra = (360 - OFFSET) - angle_lidar, nen tang OFFSET
// len bao nhieu la keo bearing xuong bay nhieu. Do duoc du 90 -> +90.
//
// Loi nay KHONG lam meo ban do (ban do va anh quet van nhat quan voi nhau,
// chi cung bi xoay so voi xe) nhung voi Nav2 thi la tai hoa: no se thay vat
// can lech di dung mot goc vuong, ne sang cho trong va dam vao cho co vat.
//
// Do lai neu thao lap lai LiDAR. Chi sua o DAY, dung sua them rpy cua
// lidar_joint trong URDF - lam ca hai la bu thua gap doi.
#define LIDAR_ANGLE_OFFSET_DEG 180.0f

// Tran kich thuoc message do thu vien micro_ros_arduino quyet dinh luc bien
// dich (src/uxr/client/config.h), khong sua duoc tu sketch.
//   Message <= MTU -> BEST_EFFORT, 1 datagram, khong ACK -> full 8.5 Hz
//   Message >  MTU -> bat buoc RELIABLE + phan manh -> tut xuong ~3 Hz, RViz giat
//   512  = ban precompiled goc              -> toi da ~107 bin
//   1472 = sau khi chay rebuild_microros_mtu.sh -> toi da ~348 bin
#define MICROROS_MTU        1472
#define SCAN_BINS           320      // 1.125 do/tia
#define PUBLISH_INTENSITIES 0

#define EST_MSG_BYTES (80 + 4 * SCAN_BINS * (1 + PUBLISH_INTENSITIES))
#if EST_MSG_BYTES > MICROROS_MTU
  #error "LaserScan vuot 1 datagram -> phai dung reliable va tut ~3 Hz. Giam SCAN_BINS hoac nang MICROROS_MTU."
#endif

#define RANGE_MIN_M 0.10f
#define RANGE_MAX_M 8.00f

#define LIDAR_PWM_FREQ    10000
#define LIDAR_PWM_BITS    11
#define LIDAR_PWM_CHANNEL 2

// ============================================================================
//  5. BIEN TOAN CUC
// ============================================================================
#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { \
    Serial.printf("microROS error %d @ line %d\n", (int)rc, __LINE__); } }
#define RCSOFTCHECK(fn) { (void)(fn); }

// --- micro-ROS ---
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
rclc_executor_t executor;

rcl_subscription_t cmd_vel_sub;
rcl_publisher_t    scan_pub, imu_pub, js_pub;
#if ODOM_ON_ESP32
rcl_publisher_t    odom_pub, tf_pub;
#endif

geometry_msgs__msg__Twist       cmd_vel_msg;
sensor_msgs__msg__LaserScan     scan_msg;
sensor_msgs__msg__Imu           imu_msg;
sensor_msgs__msg__JointState    js_msg;
#if ODOM_ON_ESP32
nav_msgs__msg__Odometry         odom_msg;
tf2_msgs__msg__TFMessage        tf_msg;
geometry_msgs__msg__TransformStamped tf_transforms[1];
#endif

// Bo nho tinh cho /joint_states. Ten khop PHAI khop robot.urdf.xacro, neu
// khong robot_state_publisher se bo qua va banh xe trong RViz khong quay.
static char  js_name_l[20] = "left_wheel_joint";
static char  js_name_r[20] = "right_wheel_joint";
static rosidl_runtime_c__String js_names[2];
static double js_position[2];      // rad, TICH LUY tu luc khoi dong
static double js_velocity[2];      // rad/s

// Bo dem tinh cho cac truong chuoi (micro-ROS khong tu cap phat)
static char odom_frame_buf[16]  = ODOM_FRAME;
static char base_frame_buf[24]  = BASE_FRAME;
static char laser_frame_buf[24] = LASER_FRAME;
static char imu_frame_buf[16]   = IMU_FRAME;
static char tf_odom_buf[16]     = ODOM_FRAME;
static char tf_base_buf[24]     = BASE_FRAME;

// --- LiDAR ---
HardwareSerial LidarSerial(2);
LDS_YDLIDAR_X2_X2L lidar;

static float scan_ranges[SCAN_BINS];
static float scan_intens[SCAN_BINS];
static float bin_dist[2][SCAN_BINS];      // ping-pong giua 2 core
static float bin_qual[2][SCAN_BINS];
static volatile uint8_t fill_idx = 0, ready_idx = 1;
static volatile bool    scan_ready = false;
static volatile float   scan_freq_hz = 7.0f;
static SemaphoreHandle_t buf_mutex;
static volatile uint32_t scans_done = 0, points_in_scan = 0, points_last_scan = 0;
static volatile uint32_t publish_failed = 0;

// --- Do luong de tim nut co chai (in ra Serial moi giay) ---
// ros_task chi day duoc ~4 message/giay ra khoi chip du rcl_publish luon tra
// ve OK. Dem rieng tung khau de biet cham o dau chu khong doan.
static volatile uint32_t ros_loops  = 0;   // so vong ros_task / giay
static volatile uint32_t pub_scan   = 0;   // so lan publish OK / giay
static volatile uint32_t pub_odom   = 0;
static volatile uint32_t pub_imu    = 0;
static volatile uint32_t t_imu_max  = 0;   // us - lau nhat 1 lan doc+gui IMU
static volatile uint32_t t_pub_max  = 0;   // us - lau nhat 1 lan gui odom+tf
static volatile uint32_t t_scan_max = 0;   // us - lau nhat 1 lan gui scan
static volatile uint32_t t_loop_max = 0;   // us - lau nhat 1 vong ros_task
static volatile uint32_t t_spin_max = 0;   // us - lau nhat 1 lan spin_some
static volatile uint32_t t_sync_max = 0;   // us - lau nhat 1 lan sync_session+ping

// --- IMU ---
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDR, &Wire);
static volatile bool  imu_ok = false;
static volatile float gyro_z_rps = 0.0f;   // rad/s, motor_task doc de tich phan yaw
static float gyro_z_bias = 0.0f;           // rad/s, do luc khoi dong roi tru di

// --- Dong co / encoder ---
ESP32Encoder encL, encR;

struct Wheel {
  ESP32Encoder *enc;
  int  pin_in1, pin_in2, pin_pwm;
  bool invert_motor, invert_enc;

  int   last_direction  = 0;      // 1 thuan, -1 nghich, 0 dung
  long  prev_count      = 0;
  // Tong xung TICH LUY tu luc khoi dong, DA ap dung invert_enc. Day la thu
  // duoc gui len laptop: mat goi khong ton gi vi goi sau mang ca gia tri.
  long long total_counts = 0;
  float target_rpm      = 0.0f;   // lenh tu /cmd_vel
  float ramped_rpm      = 0.0f;
  float current_rpm     = 0.0f;   // sau loc
  float previous_rpm    = 0.0f;
  float integral        = 0.0f;
  int   output_pwm      = 0;
  float dist_delta_m    = 0.0f;   // quang duong chu ky vua roi, cho odom

  unsigned long wrong_dir_since = 0;
  unsigned long stall_since     = 0;
};
Wheel wl, wr;

static volatile bool emergency_stop = false;
static volatile unsigned long last_cmd_vel_ms = 0;

// --- Odometry ---
static volatile float odom_x = 0.0f, odom_y = 0.0f, odom_th = 0.0f;
static volatile float odom_v = 0.0f, odom_w = 0.0f;
static portMUX_TYPE odom_mux = portMUX_INITIALIZER_UNLOCKED;

// ============================================================================
//  6. DIEU KHIEN DONG CO  (TB6612FNG / L298N)
// ============================================================================
static void wheel_write_pwm(Wheel &w, int pwm) {
  if (w.invert_motor) pwm = -pwm;
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);

  int direction = (pwm > 0) - (pwm < 0);

  // Doi chieu thi PHAI ha ENA ve 0 TRUOC khi lat IN1/IN2, neu khong ca hai
  // nhanh cau H dan cung luc trong khoanh khac chuyen mach -> soc dong, nong
  // IC va sut nguon. Lay tu motor_1_wheel_l298n.ino.
  if (direction != w.last_direction) {
    ledcWrite(w.pin_pwm, 0);
    if (direction > 0) {
      digitalWrite(w.pin_in1, HIGH); digitalWrite(w.pin_in2, LOW);
    } else if (direction < 0) {
      digitalWrite(w.pin_in1, LOW);  digitalWrite(w.pin_in2, HIGH);
    } else {
      digitalWrite(w.pin_in1, LOW);  digitalWrite(w.pin_in2, LOW);   // tha troi
    }
    w.last_direction = direction;
  }

  ledcWrite(w.pin_pwm, abs(pwm));
}

static void trigger_estop(const char *reason) {
  emergency_stop = true;
  wl.target_rpm = wr.target_rpm = 0.0f;
  wl.ramped_rpm = wr.ramped_rpm = 0.0f;
  wl.integral   = wr.integral   = 0.0f;
  wl.output_pwm = wr.output_pwm = 0;
  wheel_write_pwm(wl, 0);
  wheel_write_pwm(wr, 0);
  Serial.printf("\n!!! ESTOP: %s\n", reason);
}

// Bao ve sai chieu (dao nham day) va ket banh / mat encoder
static void wheel_check_protection(Wheel &w, const char *name, unsigned long now) {
  bool wrong_dir = abs(w.output_pwm) >= WRONG_DIR_PWM &&
                   ((w.output_pwm > 0 && w.current_rpm < -WRONG_DIR_RPM) ||
                    (w.output_pwm < 0 && w.current_rpm >  WRONG_DIR_RPM));
  if (wrong_dir) {
    if (w.wrong_dir_since == 0) w.wrong_dir_since = now;
    else if (now - w.wrong_dir_since > WRONG_DIR_TIME_MS) {
      Serial.printf("Banh %s quay nguoc chieu PWM. Kiem tra INVERT_MOTOR_x / INVERT_ENC_x.\n", name);
      trigger_estop("Banh quay nguoc chieu");
      return;
    }
  } else w.wrong_dir_since = 0;

  bool stalled = abs(w.output_pwm) >= STALL_PWM && fabsf(w.current_rpm) < STALL_RPM;
  if (stalled) {
    if (w.stall_since == 0) w.stall_since = now;
    else if (now - w.stall_since > STALL_TIME_MS) {
      Serial.printf("Banh %s ket hoac mat tin hieu encoder.\n", name);
      trigger_estop("Banh ket / mat encoder");
    }
  } else w.stall_since = 0;
}

// Mot chu ky PID cua mot banh. Tra ve quang duong di duoc trong chu ky (m).
static void wheel_control_step(Wheel &w, const char *name, float dt) {
  // --- 1. Doc encoder ---
  long count = (long)w.enc->getCount();
  long delta = count - w.prev_count;
  w.prev_count = count;
  if (w.invert_enc) delta = -delta;
  w.total_counts += delta;

  float revs   = (float)delta / ENC_PULSES_PER_REV;
  float rawRPM = revs * (60.0f / dt);
  w.dist_delta_m = revs * 2.0f * (float)M_PI * WHEEL_RADIUS_M;

  // --- 2. Loc thong thap ---
  w.current_rpm = lpfAlpha * rawRPM + (1.0f - lpfAlpha) * w.current_rpm;

  // --- 3. Tinh PWM ---
  if (emergency_stop) {
    w.ramped_rpm = 0.0f; w.integral = 0.0f; w.output_pwm = 0;
  } else {
    float ramp_step = maxAccelRPMPerSec * dt;
    if (w.ramped_rpm < w.target_rpm)
      w.ramped_rpm = fminf(w.ramped_rpm + ramp_step, w.target_rpm);
    else if (w.ramped_rpm > w.target_rpm)
      w.ramped_rpm = fmaxf(w.ramped_rpm - ramp_step, w.target_rpm);

    if (fabsf(w.target_rpm) < 0.001f && fabsf(w.ramped_rpm) < 0.001f) {
      w.integral = 0.0f;
      w.output_pwm = 0;
    } else {
      float error = w.ramped_rpm - w.current_rpm;

      // Chong bao hoa tich phan
      bool sat_hi = w.output_pwm >=  PWM_MAX && error > 0.0f;
      bool sat_lo = w.output_pwm <= -PWM_MAX && error < 0.0f;
      if (!sat_hi && !sat_lo) w.integral += Ki * error;
      w.integral = constrain(w.integral, -(float)PWM_MAX, (float)PWM_MAX);

      // Vi phan theo gia tri DO (khong theo sai so) -> khong giat khi doi setpoint
      float d_input = w.current_rpm - w.previous_rpm;

      float ff = 0.0f;
      if      (w.ramped_rpm >  0.5f) ff = Kf * w.ramped_rpm + deadbandPWM;
      else if (w.ramped_rpm < -0.5f) ff = Kf * w.ramped_rpm - deadbandPWM;

      float out = ff + Kp * error + w.integral - Kd * d_input;
      w.output_pwm = constrain((int)out, -PWM_MAX, PWM_MAX);
    }
  }

  w.previous_rpm = w.current_rpm;
  if (!emergency_stop) wheel_check_protection(w, name, millis());
  wheel_write_pwm(w, w.output_pwm);
}

// ============================================================================
//  7. ODOMETRY
// ============================================================================
static void odom_update(float dt) {
  float d_l = wl.dist_delta_m;
  float d_r = wr.dist_delta_m;
  float d_center = 0.5f * (d_l + d_r);

#if USE_GYRO_FOR_YAW
  // Gyro chinh xac hon han encoder o phep quay: khong bi anh huong boi truot
  // banh, sai lech ban kinh hay khe ho co khi.
  float d_theta = gyro_z_rps * dt;
#else
  float d_theta = (d_r - d_l) / WHEEL_SEPARATION_M;
#endif

  // Tich phan runge-kutta bac 2: lay goc o GIUA chu ky, chinh xac hon
  // Euler thuan khi xe vua chay vua quay.
  float th_mid = odom_th + 0.5f * d_theta;

  portENTER_CRITICAL(&odom_mux);
  odom_x  += d_center * cosf(th_mid);
  odom_y  += d_center * sinf(th_mid);
  odom_th += d_theta;
  if      (odom_th >  (float)M_PI) odom_th -= 2.0f * (float)M_PI;
  else if (odom_th < -(float)M_PI) odom_th += 2.0f * (float)M_PI;
  odom_v = d_center / dt;
  odom_w = d_theta  / dt;
  portEXIT_CRITICAL(&odom_mux);
}

// ============================================================================
//  8. TASK DIEU KHIEN DONG CO  (Core 0, uu tien 3, 50 Hz)
// ============================================================================
void motor_task(void *pv) {
  (void)pv;
  const TickType_t period = pdMS_TO_TICKS(PID_INTERVAL_MS);
  TickType_t last_wake = xTaskGetTickCount();
  const float dt = PID_INTERVAL_MS / 1000.0f;

  for (;;) {
    // Mat lenh dieu khien qua lau -> dung xe. Nav2 treo, Wi-Fi rot hay agent
    // chet deu roi vao day. Khong co cai nay thi xe chay tiep den khi dam.
    if (millis() - last_cmd_vel_ms > CMD_VEL_TIMEOUT_MS) {
      wl.target_rpm = 0.0f;
      wr.target_rpm = 0.0f;
    }

    wheel_control_step(wl, "TRAI", dt);
    wheel_control_step(wr, "PHAI", dt);
    odom_update(dt);

    vTaskDelayUntil(&last_wake, period);
  }
}

// ============================================================================
//  9. CALLBACK LiDAR  (chay tren Core 1)
// ============================================================================
static void clear_buffer(uint8_t idx) {
  memset((void *)bin_dist[idx], 0, sizeof(bin_dist[idx]));
  memset((void *)bin_qual[idx], 0, sizeof(bin_qual[idx]));
}

int    lidar_serial_read_callback()  { return LidarSerial.read(); }
size_t lidar_serial_write_callback(const uint8_t *b, size_t n) { return LidarSerial.write(b, n); }

void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality,
                               bool scan_completed) {
  // scan_completed == true: diem NAY mo dau vong quet moi -> dong goi vong cu
  if (scan_completed) {
    float f = lidar.getCurrentScanFreqHz();
    if (f > 1.0f && f < 20.0f) scan_freq_hz = f;

    if (xSemaphoreTake(buf_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      ready_idx  = fill_idx;
      fill_idx  ^= 1;
      scan_ready = true;
      xSemaphoreGive(buf_mutex);
      clear_buffer(fill_idx);
    }
    scans_done++;
    points_last_scan = points_in_scan;
    points_in_scan = 0;
  }

  float d_m = distance_mm * 0.001f;
  if (d_m < RANGE_MIN_M || d_m > RANGE_MAX_M) return;

  float a = angle_deg + LIDAR_ANGLE_OFFSET_DEG;
#if LIDAR_ANGLE_FLIP
  a = 360.0f - a;
#endif
  int bin = (int)lroundf(a * (float)SCAN_BINS / 360.0f) % SCAN_BINS;
  if (bin < 0) bin += SCAN_BINS;

  uint8_t i = fill_idx;
  // Nhieu diem cung mot bin -> giu diem GAN nhat (an toan cho tranh vat can)
  if (bin_dist[i][bin] == 0.0f || d_m < bin_dist[i][bin]) {
    bin_dist[i][bin] = d_m;
    bin_qual[i][bin] = quality;
  }
  points_in_scan++;
}

void lidar_packet_callback(uint8_t *p, uint16_t n, bool done) { (void)p; (void)n; (void)done; }
void lidar_info_callback(LDS::info_t c, String s)  { Serial.printf("LiDAR info %s: %s\n",  lidar.infoCodeToString(c).c_str(), s.c_str()); }
void lidar_error_callback(LDS::result_t c, String s){ Serial.printf("LiDAR error %s: %s\n", lidar.resultCodeToString(c).c_str(), s.c_str()); }

void lidar_motor_pin_callback(float value, LDS::lds_pin_t lidar_pin) {
  int pin = (lidar_pin == LDS::LDS_MOTOR_EN_PIN) ? LIDAR_EN_PIN : LIDAR_PWM_PIN;
  // Chan dat -1 la chan khong noi day -> bo qua, dung goi pinMode(-1, ...).
  // Chan EN roi vao day voi LiDAR 4 chan; neu doi sang YDLIDAR X4 (co dung EN)
  // thi phai gan LIDAR_EN_PIN mot GPIO that.
  if (pin < 0) return;
  if (value <= (float)LDS::DIR_INPUT) {
    if (value == (float)LDS::DIR_OUTPUT_PWM) {
      #if ESP_IDF_VERSION_MAJOR < 5
      ledcSetup(LIDAR_PWM_CHANNEL, LIDAR_PWM_FREQ, LIDAR_PWM_BITS);
      ledcAttachPin(pin, LIDAR_PWM_CHANNEL);
      #else
      ledcAttachChannel(pin, LIDAR_PWM_FREQ, LIDAR_PWM_BITS, LIDAR_PWM_CHANNEL);
      #endif
    } else
      pinMode(pin, (value == (float)LDS::DIR_INPUT) ? INPUT : OUTPUT);
    return;
  }
  if (value < (float)LDS::VALUE_PWM)
    digitalWrite(pin, (value == (float)LDS::VALUE_HIGH) ? HIGH : LOW);
  else {
    int pwm_value = ((1 << LIDAR_PWM_BITS) - 1) * value;
    #if ESP_IDF_VERSION_MAJOR < 5
    ledcWrite(LIDAR_PWM_CHANNEL, pwm_value);
    #else
    ledcWriteChannel(LIDAR_PWM_CHANNEL, pwm_value);
    #endif
  }
}

// ============================================================================
//  10. micro-ROS: NHAN /cmd_vel
// ============================================================================
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;

  float v = (float)m->linear.x;    // m/s
  float w = (float)m->angular.z;   // rad/s

  // Dong hoc nghich cua xe vi sai
  float v_l = v - w * WHEEL_SEPARATION_M * 0.5f;
  float v_r = v + w * WHEEL_SEPARATION_M * 0.5f;

  // m/s -> RPM
  const float mps_to_rpm = 60.0f / (2.0f * (float)M_PI * WHEEL_RADIUS_M);
  float rpm_l = v_l * mps_to_rpm;
  float rpm_r = v_r * mps_to_rpm;

  // Vuot tran thi ha CA HAI banh theo cung ty le, giu nguyen ban kinh cua
  // duong cong. Cat rieng tung banh se lam xe re sai huong.
  float peak = fmaxf(fabsf(rpm_l), fabsf(rpm_r));
  if (peak > MAX_WHEEL_RPM) {
    float k = MAX_WHEEL_RPM / peak;
    rpm_l *= k;
    rpm_r *= k;
  }

  if (!emergency_stop) {
    wl.target_rpm = rpm_l;
    wr.target_rpm = rpm_r;
  }
  last_cmd_vel_ms = millis();
}

// ============================================================================
//  11. micro-ROS: PHAT
// ============================================================================
// KHONG dung rmw_uros_epoch_nanos() truc tiep lam timestamp.
// Da do bang cach in stamp tho cua /odom ve phia ROS: truong nanosec DUNG YEN
// (vd 249635742 suot hon 10 giay lien), chi sec tang; no chi doi dung vao luc
// rmw_uros_sync_session() chay. Tuc dong ho ben trong ham do co do phan giai
// ~1 giay, trong khi 50 message/giay deu can stamp rieng. Neu de nguyen thi
// AMCL, TF va costmap deu thay 50 message "cung mot thoi diem" -> hong.
//
// Cach lam: lay MOT moc epoch tu agent (sync_session do round-trip nen moc nay
// chinh xac duoi giay), roi tu do tu chay bang esp_timer_get_time() - dong ho
// us cua ESP32, 64 bit, khong tran.
static int64_t epoch_anchor_ns = 0;   // epoch tai thoi diem lay moc
static int64_t local_anchor_us = 0;   // esp_timer_get_time() tai thoi diem do

// Cho den dung khoanh khac dong ho tho nhay sang gia tri moi. Tra ve so ms da
// cho, hoac -1 neu het han ma khong thay nhay. Do duoc tren phan cung: buoc
// nhay 763 ms -> dung la dong ho 1 giay.
static int32_t wait_for_epoch_tick(uint32_t limit_ms) {
  int64_t e0 = rmw_uros_epoch_nanos();
  uint32_t t0 = millis();
  while (millis() - t0 < limit_ms) {
    if (rmw_uros_epoch_nanos() != e0) return (int32_t)(millis() - t0);
    delay(1);
  }
  return -1;
}

// Goi trong setup(), luc chua publish gi nen block vai giay la vo hai.
//
// Khong chi gia tri tra ve bi lam tron: CHINH cai offset ma sync_session tinh
// ra cung bi. No lay offset = ((t2-t1)+(t3-t4))/2 voi t1, t4 doc tu dong ho
// tho 1 giay do, nen offset sai bao nhieu thi moi stamp sai bay nhieu. Da do
// tu phia ROS: lan dau chi neo dung canh tick ma khong lo cho sync, stamp tre
// 1.158 s so voi dong ho may -> Nav2 loai het vi qua cu.
//
// Dong ho tho chi DUNG ngay khoanh khac no vua nhay. Vi vay:
//   1. cho tick  -> t1 cua sync duoc doc khi dong ho tho vua dung
//   2. sync ngay -> offset chi con sai co RTT (vai ms)
//   3. cho tick tiep -> neo moc vao dung canh, sai so lam tron lai bang 0
static void clock_anchor_bootstrap() {
  int32_t w1 = wait_for_epoch_tick(1500);
  rcl_ret_t rc = rmw_uros_sync_session(1000);
  int32_t w2 = wait_for_epoch_tick(1500);

  epoch_anchor_ns = rmw_uros_epoch_nanos();
  local_anchor_us = esp_timer_get_time();
  Serial.printf("Dong ho: tick1 %ld ms, sync -> %d, tick2 %ld ms -> neo theo esp_timer\n",
                (long)w1, (int)rc, (long)w2);
  if (w1 < 0 || w2 < 0)
    Serial.println("  CANH BAO: khong bat duoc canh tick -> stamp co the lech toi 1 giay");
}

// Bu troi thach anh, chay dinh ky. Phai canh tick y het luc khoi tao: doc
// epoch o giua hai tick la tu tay bom vao sai so toi 1 giay.
// Khong nhay thang den gia tri moi vi stamp giat LUI la loi nang voi TF/AMCL;
// chi keo 1/5 khoang lech moi lan.
static void clock_resync() {
  if (wait_for_epoch_tick(1500) < 0) return;
  if (rmw_uros_sync_session(200) != RMW_RET_OK) return;
  if (wait_for_epoch_tick(1500) < 0) return;

  int64_t e = rmw_uros_epoch_nanos();
  if (e < 1000000000LL) return;
  int64_t now_us    = esp_timer_get_time();
  int64_t predicted = epoch_anchor_ns + (now_us - local_anchor_us) * 1000LL;
  epoch_anchor_ns   = predicted + (e - predicted) / 5;
  local_anchor_us   = now_us;
}

static void fill_stamp(builtin_interfaces__msg__Time *stamp) {
  // Chua neo duoc (chua ket noi agent) -> dung dong ho noi bo, Nav2 se bo qua
  // vi lech epoch, nhung it nhat khong sai nhip.
  int64_t t_ns = (epoch_anchor_ns > 0)
                 ? epoch_anchor_ns + (esp_timer_get_time() - local_anchor_us) * 1000LL
                 : (int64_t)esp_timer_get_time() * 1000LL;
  stamp->sec     = (int32_t)(t_ns / 1000000000LL);
  stamp->nanosec = (uint32_t)(t_ns % 1000000000LL);
}

static void publish_scan_if_ready() {
  if (!scan_ready) return;
  if (xSemaphoreTake(buf_mutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
  uint8_t idx = ready_idx;
  memcpy(scan_ranges, (const void *)bin_dist[idx], sizeof(scan_ranges));
  memcpy(scan_intens, (const void *)bin_qual[idx], sizeof(scan_intens));
  scan_ready = false;
  xSemaphoreGive(buf_mutex);

  float freq = (scan_freq_hz > 1.0f) ? scan_freq_hz : 7.0f;
  fill_stamp(&scan_msg.header.stamp);
  scan_msg.angle_min       = 0.0f;
  scan_msg.angle_increment = 2.0f * (float)M_PI / (float)SCAN_BINS;
  scan_msg.angle_max       = 2.0f * (float)M_PI - scan_msg.angle_increment;
  scan_msg.scan_time       = 1.0f / freq;
  scan_msg.time_increment  = scan_msg.scan_time / (float)SCAN_BINS;
  scan_msg.range_min       = RANGE_MIN_M;
  scan_msg.range_max       = RANGE_MAX_M;
  // range == 0 nam ngoai [range_min, range_max] -> Nav2/RViz tu bo qua tia rong

  uint32_t t0 = micros();
  if (rcl_publish(&scan_pub, &scan_msg, NULL) != RCL_RET_OK) publish_failed++;
  else pub_scan++;
  uint32_t dt = micros() - t0;
  if (dt > t_scan_max) t_scan_max = dt;
}

// Phat goc banh TICH LUY (rad) + toc do goc (rad/s). Day la toan bo nhung gi
// laptop can de tu tinh odometry, va cung dung luon lam /joint_states that cho
// robot_state_publisher - truoc day phai dung joint_state_publisher bom goc 0
// gia, nen banh trong RViz khong bao gio quay.
//
// Gui GOC chu khong gui quang duong: WHEEL_RADIUS_M khong tham gia, nen hieu
// chinh ban kinh lan chi la sua tham so ROS tren laptop, khong phai nap chip.
static void publish_joint_states() {
  const double CNT2RAD = 2.0 * M_PI / (double)ENC_PULSES_PER_REV;
  const double RPM2RPS = 2.0 * M_PI / 60.0;

  uint32_t t0 = micros();
  fill_stamp(&js_msg.header.stamp);
  js_position[0] = (double)wl.total_counts * CNT2RAD;
  js_position[1] = (double)wr.total_counts * CNT2RAD;
  js_velocity[0] = (double)wl.current_rpm * RPM2RPS;
  js_velocity[1] = (double)wr.current_rpm * RPM2RPS;
  if (rcl_publish(&js_pub, &js_msg, NULL) != RCL_RET_OK) publish_failed++;
  else pub_odom++;              // dung chung bo dem voi odom cho dong log gon
  uint32_t dt = micros() - t0;
  if (dt > t_pub_max) t_pub_max = dt;
}

#if ODOM_ON_ESP32
static void publish_odom_and_tf() {
  float x, y, th, v, w;
  portENTER_CRITICAL(&odom_mux);
  x = odom_x; y = odom_y; th = odom_th; v = odom_v; w = odom_w;
  portEXIT_CRITICAL(&odom_mux);

  // Chi co yaw -> quaternion rut gon
  float qz = sinf(th * 0.5f);
  float qw = cosf(th * 0.5f);

  uint32_t t0 = micros();
  fill_stamp(&odom_msg.header.stamp);
  odom_msg.pose.pose.position.x    = x;
  odom_msg.pose.pose.position.y    = y;
  odom_msg.pose.pose.position.z    = 0.0;
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = qz;
  odom_msg.pose.pose.orientation.w = qw;
  odom_msg.twist.twist.linear.x    = v;
  odom_msg.twist.twist.angular.z   = w;
  if (rcl_publish(&odom_pub, &odom_msg, NULL) != RCL_RET_OK) publish_failed++;
  else pub_odom++;

  // TF odom -> base_footprint (thay cho tf_topic cua plugin DiffDrive)
  tf_transforms[0].header.stamp            = odom_msg.header.stamp;
  tf_transforms[0].transform.translation.x = x;
  tf_transforms[0].transform.translation.y = y;
  tf_transforms[0].transform.translation.z = 0.0;
  tf_transforms[0].transform.rotation.x    = 0.0;
  tf_transforms[0].transform.rotation.y    = 0.0;
  tf_transforms[0].transform.rotation.z    = qz;
  tf_transforms[0].transform.rotation.w    = qw;
  if (rcl_publish(&tf_pub, &tf_msg, NULL) != RCL_RET_OK) publish_failed++;
  uint32_t dt = micros() - t0;
  if (dt > t_pub_max) t_pub_max = dt;
}
#endif  // ODOM_ON_ESP32

// Do do lech khong cua gyro Z luc khoi dong, luc xe chac chan dung yen.
//
// odom_update() tich phan gyro_z de ra goc quay (USE_GYRO_FOR_YAW = 1), va
// phep tich phan khong tha thu cho bias: lech 0.005 rad/s la odom tu quay
// them 17 do moi phut du xe khong nhuc nhich. Dung nhu trieu chung "chay dau
// thi khop, cang lau cang lech, quay ve cho cu thi sai vi tri".
//
// Do moi lan khoi dong chu khong de hang so chet, vi bias cua MEMS troi theo
// nhiet do - lanh luc moi bat va am dan khi chay.
static void calibrate_gyro_bias() {
  if (!imu_ok) return;
  const int N = 200;                    // 200 mau x 10 ms = 2 giay
  const float DEG2RAD = (float)M_PI / 180.0f;
  double sum = 0.0, sum2 = 0.0;
  for (int i = 0; i < N; i++) {
    float w = (float)bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE).z() * DEG2RAD;
    sum  += w;
    sum2 += (double)w * w;
    delay(10);
  }
  float mean = (float)(sum / N);
  float sd   = sqrtf(fmaxf(0.0f, (float)(sum2 / N - (double)mean * mean)));

  // Neu luc do xe bi dong day thi so trung binh khong phai bias. Tru nham mot
  // so sai con te hon khong tru. Nguong 0.05 rad/s = 2.9 do/s, cao hon han
  // nhieu cua BNO055 luc dung yen.
  if (sd > 0.05f) {
    Serial.printf("Bias gyro: KHONG do duoc, xe dang bi dong day (sd %.4f rad/s)."
                  " Giu bias = 0.\n", sd);
    return;
  }
  gyro_z_bias = mean;
  Serial.printf("Bias gyro Z: %+.5f rad/s (%+.3f do/s), nhieu %.5f -> da tru\n",
                gyro_z_bias, gyro_z_bias / DEG2RAD, sd);
}

static void read_and_publish_imu() {
  if (!imu_ok) return;
  uint32_t t0 = micros();

  // getVector tra ve deg/s cho gyro va m/s^2 cho gia toc.
  imu::Vector<3> g = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> a = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Quaternion q = bno.getQuat();

  const float DEG2RAD = (float)M_PI / 180.0f;
  // Tru bias: day la gia tri motor_task tich phan ra goc quay, nen no PHAI
  // bang 0 khi xe dung yen. x/y khong tru vi odometry khong dung den.
  gyro_z_rps = (float)g.z() * DEG2RAD - gyro_z_bias;

  fill_stamp(&imu_msg.header.stamp);
  imu_msg.orientation.w = q.w();
  imu_msg.orientation.x = q.x();
  imu_msg.orientation.y = q.y();
  imu_msg.orientation.z = q.z();
  imu_msg.angular_velocity.x = (float)g.x() * DEG2RAD;
  imu_msg.angular_velocity.y = (float)g.y() * DEG2RAD;
  imu_msg.angular_velocity.z = gyro_z_rps;
  imu_msg.linear_acceleration.x = a.x();
  imu_msg.linear_acceleration.y = a.y();
  imu_msg.linear_acceleration.z = a.z();
  if (rcl_publish(&imu_pub, &imu_msg, NULL) != RCL_RET_OK) publish_failed++;
  else pub_imu++;
  uint32_t dt = micros() - t0;
  if (dt > t_imu_max) t_imu_max = dt;   // gom ca 3 lan doc I2C
}

// ============================================================================
//  12. TASK micro-ROS  (Core 0, uu tien 1)
// ============================================================================
void ros_task(void *pv) {
  (void)pv;
  uint32_t last_sync = 0, last_ping = 0, last_odom = 0, last_imu = 0;
  int ping_fail = 0;

  for (;;) {
    uint32_t loop_t0 = micros();
    ros_loops++;
    // timeout 0 = poll khong chan (quy uoc rcl: am = cho mai mai, 0 = khong cho).
    // RCL_MS_TO_NS(1) da duoc thu va van chan ~1 giay.
    rclc_executor_spin_some(&executor, 0);
    uint32_t spin_dt = micros() - loop_t0;
    if (spin_dt > t_spin_max) t_spin_max = spin_dt;
    publish_scan_if_ready();

    uint32_t now = millis();

    // Nhip 50 Hz. Dung "last += 20" chu khong "last = now": gan bang now thi
    // phan vuot qua 20 ms bi nuot mat va lich tu troi cham lai. Da do tu phia
    // ROS: khoang cach dau thoi gian co dinh o 20 ms nhung duoi keo toi 32 ms,
    // trung binh 22.6 ms = 44 Hz thay vi 50.
    // Neu tut qua xa (vong lap bi chan lau) thi dat lai, dung don cuc mot lan.
    if (now - last_odom >= 20) {
      last_odom += 20;
      if (now - last_odom > 100) last_odom = now;
#if ODOM_ON_ESP32
      publish_odom_and_tf();
#endif
      publish_joint_states();
    }
    if (now - last_imu >= 20) {
      last_imu += 20;
      if (now - last_imu > 100) last_imu = now;
      read_and_publish_imu();
    }

    // sync_session va ping_agent deu CHAN, va do duoc la chung KHONG ton trong
    // timeout minh dat: ping_agent(150, 1) chan 127 ms, 228 ms, va mot lan
    // 1050 ms - giay do mat trang du lieu (ros_task 0 vong/s, pub/s 0 0 0).
    //
    // Nhung khi Nav2 (hoac tay cam) dang dieu khien thi /cmd_vel ve lien tuc,
    // va viec no ve TU NO da chung minh agent con song -> ping la vo nghia.
    // Nen chi chay hai lenh nay khi da mat /cmd_vel qua han, tuc xe da duoc
    // lenh dung: luc do co chan ca giay cung khong ai thiet.
    //
    // Agent chet giua duong van xu ly dung: /cmd_vel tat -> motor_task dung xe
    // sau 500 ms -> moi bat dau ping -> 5 lan that bai (25 s) thi restart.
    bool idle = (now - last_cmd_vel_ms) > CMD_VEL_TIMEOUT_MS;

    uint32_t t_sync0 = micros();
    if (idle && now - last_sync > 60000) {
      last_sync = now;
      clock_resync();
    }

    // Mat agent qua lau -> khoi dong lai cho sach.
    // Dung dong co TRUOC khi restart, neu khong xe lao tu do luc ESP32 boot lai.
    if (idle && now - last_ping > 5000) {
      last_ping = now;
      if (rmw_uros_ping_agent(150, 1) != RMW_RET_OK) {
        if (++ping_fail >= 5) {
          Serial.println("Mat ket noi agent -> dung dong co va restart");
          wheel_write_pwm(wl, 0);
          wheel_write_pwm(wr, 0);
          delay(100);
          ESP.restart();
        }
      } else ping_fail = 0;
    }
    if (!idle) {
      // Co /cmd_vel ve = agent con song. Day moc ping lui de mot khoang lang
      // ngan (nha tay cam ra chang han) khong lap tuc keo theo mot lan chan.
      ping_fail = 0;
      last_ping = now;
    }
    uint32_t sync_dt = micros() - t_sync0;   // gop sync + ping: ca hai deu chan
    if (sync_dt > t_sync_max) t_sync_max = sync_dt;

    uint32_t loop_dt = micros() - loop_t0;
    if (loop_dt > t_loop_max) t_loop_max = loop_dt;

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// ============================================================================
//  13. KHOI TAO micro-ROS
// ============================================================================
static void init_microros() {
  set_microros_wifi_transports((char *)WIFI_SSID, (char *)WIFI_PASS,
                               (char *)AGENT_IP, AGENT_PORT);
  esp_wifi_set_ps(WIFI_PS_NONE);   // tat power-save: giam jitter / mat goi UDP

  Serial.print("Dang ket noi Wi-Fi");
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries++ < 40) { delay(500); Serial.print("."); }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("LOI: khong ket noi duoc Wi-Fi -> restart");
    delay(1000);
    ESP.restart();
  }
  Serial.print("Wi-Fi OK, IP xe: "); Serial.println(WiFi.localIP());

  Serial.printf("Cho micro_ros_agent tai %s:%d", AGENT_IP, AGENT_PORT);
  while (rmw_uros_ping_agent(500, 1) != RMW_RET_OK) { Serial.print("."); delay(500); }
  Serial.println("\nDa thay agent!");

  char node_name[32];
  snprintf(node_name, sizeof(node_name), "esp32_agv_base_%d", ROBOT_ID);

  allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
  rmw_uros_options_set_client_key(0x11111111 * ROBOT_ID, rmw_options);

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, node_name, "", &support));

  // QoS PHAI khop ben nhan, khong thi ROS 2 KHONG chuyen mot message nao va
  // cung khong bao loi o cho de thay. Da dinh dung bay nay tren xe that: ca 4
  // topic deu de best-effort -> tf2 khong nhan duoc gi -> frame "odom" khong
  // ton tai -> slam_toolbox loai sach scan ("Message Filter dropping message:
  // frame 'lidar_link' ... queue is full") -> khong bao gio co ban do.
  //
  //   /scan, /imu/data : ben nhan dung SensorDataQoS = BEST_EFFORT  -> giu
  //   /tf              : tf2_ros::TransformListener luon RELIABLE   -> phai doi
  //   /odom            : nav2_util::OdomSmoother dung SystemDefaultsQoS
  //                      = RELIABLE                                 -> phai doi
  //
  // /scan giu best-effort con vi ly do khac: message <= MTU nen di gon 1
  // datagram, khong cho ACK, giu duoc dung nhip quay cua LiDAR.
  RCCHECK(rclc_publisher_init_best_effort(&scan_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "scan"));
  RCCHECK(rclc_publisher_init_best_effort(&imu_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data"));
  // /joint_states RELIABLE: ben nhan co hai, wheel_odometry.py (ta tu viet) va
  // robot_state_publisher. Publisher RELIABLE khop duoc ca subscriber
  // BEST_EFFORT lan RELIABLE; chieu nguoc lai thi khong. Nen chon RELIABLE la
  // an toan du robot_state_publisher dung QoS nao.
  RCCHECK(rclc_publisher_init_default(&js_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "joint_states"));
#if ODOM_ON_ESP32
  RCCHECK(rclc_publisher_init_default(&odom_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom"));
  RCCHECK(rclc_publisher_init_default(&tf_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), "tf"));
#endif

  RCCHECK(rclc_subscription_init_best_effort(&cmd_vel_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  // ---------------------------------------------------------------------------
  // VAN DE DA DO DUOC TREN PHAN CUNG:
  // rclc_executor_spin_some CHAN cho tới khi CO DU LIEU DEN, chu khong ton
  // trong timeout. Do duoc: spin = 965..990 ms trong khi ca vong = ~1000 ms,
  // tuc spin chiem 98%. Vi moi thu publish nam SAU spin trong cung vong lap,
  // toan bo topic tut xuong nhip cua /cmd_vel den:
  //      khong co /cmd_vel  -> /odom 0.2 Hz, /scan 0.2 Hz
  //      /cmd_vel 20 Hz     -> /odom 17.9 Hz, /scan 7.9 Hz  (dung nhip lidar)
  //
  // Dat timeout NGAY SAU init va TRUOC khi them handle: mot so ban rclc chi
  // doc executor->timeout_ns luc tao wait_set. In ma tra ve de biet co an khong.
  // ---------------------------------------------------------------------------
  rcl_ret_t rc_to = rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(1));
  Serial.printf("rclc_executor_set_timeout -> %d (%s)\n", (int)rc_to,
                rc_to == RCL_RET_OK ? "OK" : "THAT BAI");

  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,
                                         &cmd_vel_callback, ON_NEW_DATA));
  clock_anchor_bootstrap();   // tu goi sync_session ben trong, dung canh tick

  // --- Cap bo nho tinh cho cac message (micro-ROS khong tu cap phat) ---
  sensor_msgs__msg__LaserScan__init(&scan_msg);
  scan_msg.header.frame_id.data     = laser_frame_buf;
  scan_msg.header.frame_id.size     = strlen(laser_frame_buf);
  scan_msg.header.frame_id.capacity = sizeof(laser_frame_buf);
  scan_msg.ranges.data          = scan_ranges;
  scan_msg.ranges.size          = SCAN_BINS;
  scan_msg.ranges.capacity      = SCAN_BINS;
  scan_msg.intensities.data     = scan_intens;
  scan_msg.intensities.size     = PUBLISH_INTENSITIES ? SCAN_BINS : 0;
  scan_msg.intensities.capacity = SCAN_BINS;

  // /joint_states. micro-ROS khong tu cap phat nen phai tro tung con tro vao
  // bo nho tinh, ke ca mang chuoi ten khop.
  sensor_msgs__msg__JointState__init(&js_msg);
  js_names[0].data = js_name_l;
  js_names[0].size = strlen(js_name_l);
  js_names[0].capacity = sizeof(js_name_l);
  js_names[1].data = js_name_r;
  js_names[1].size = strlen(js_name_r);
  js_names[1].capacity = sizeof(js_name_r);
  js_msg.name.data     = js_names;
  js_msg.name.size     = 2;
  js_msg.name.capacity = 2;
  js_msg.position.data     = js_position;
  js_msg.position.size     = 2;
  js_msg.position.capacity = 2;
  js_msg.velocity.data     = js_velocity;
  js_msg.velocity.size     = 2;
  js_msg.velocity.capacity = 2;
  js_msg.effort.size = 0;          // khong do dong co -> khong co momen
  js_msg.header.frame_id.data     = base_frame_buf;
  js_msg.header.frame_id.size     = strlen(base_frame_buf);
  js_msg.header.frame_id.capacity = sizeof(base_frame_buf);

#if ODOM_ON_ESP32
  nav_msgs__msg__Odometry__init(&odom_msg);
  odom_msg.header.frame_id.data     = odom_frame_buf;
  odom_msg.header.frame_id.size     = strlen(odom_frame_buf);
  odom_msg.header.frame_id.capacity = sizeof(odom_frame_buf);
  odom_msg.child_frame_id.data      = base_frame_buf;
  odom_msg.child_frame_id.size      = strlen(base_frame_buf);
  odom_msg.child_frame_id.capacity  = sizeof(base_frame_buf);
  // Hiep phuong sai: encoder kha tin o x, kem o y, trung binh o yaw.
  // AMCL/EKF dung cac so nay de can nhac tin odom den dau.
  for (int i = 0; i < 36; i++) { odom_msg.pose.covariance[i] = 0.0; odom_msg.twist.covariance[i] = 0.0; }
  odom_msg.pose.covariance[0]   = 0.002;  // x
  odom_msg.pose.covariance[7]   = 0.002;  // y
  odom_msg.pose.covariance[14]  = 1e6;    // z  - xe chay tren mat phang
  odom_msg.pose.covariance[21]  = 1e6;    // roll
  odom_msg.pose.covariance[28]  = 1e6;    // pitch
  odom_msg.pose.covariance[35]  = 0.005;  // yaw
  odom_msg.twist.covariance[0]  = 0.002;
  odom_msg.twist.covariance[35] = 0.005;

  sensor_msgs__msg__Imu__init(&imu_msg);
  imu_msg.header.frame_id.data     = imu_frame_buf;
  imu_msg.header.frame_id.size     = strlen(imu_frame_buf);
  imu_msg.header.frame_id.capacity = sizeof(imu_frame_buf);
  for (int i = 0; i < 9; i++) {
    imu_msg.orientation_covariance[i]         = 0.0;
    imu_msg.angular_velocity_covariance[i]    = 0.0;
    imu_msg.linear_acceleration_covariance[i] = 0.0;
  }
  imu_msg.orientation_covariance[0] = 0.01;
  imu_msg.orientation_covariance[4] = 0.01;
  imu_msg.orientation_covariance[8] = 0.01;
  imu_msg.angular_velocity_covariance[0] = 0.001;
  imu_msg.angular_velocity_covariance[4] = 0.001;
  imu_msg.angular_velocity_covariance[8] = 0.001;
  imu_msg.linear_acceleration_covariance[0] = 0.05;
  imu_msg.linear_acceleration_covariance[4] = 0.05;
  imu_msg.linear_acceleration_covariance[8] = 0.05;

  tf2_msgs__msg__TFMessage__init(&tf_msg);
  geometry_msgs__msg__TransformStamped__init(&tf_transforms[0]);
  tf_msg.transforms.data     = tf_transforms;
  tf_msg.transforms.size     = 1;
  tf_msg.transforms.capacity = 1;
  tf_transforms[0].header.frame_id.data     = tf_odom_buf;
  tf_transforms[0].header.frame_id.size     = strlen(tf_odom_buf);
  tf_transforms[0].header.frame_id.capacity = sizeof(tf_odom_buf);
  tf_transforms[0].child_frame_id.data      = tf_base_buf;
  tf_transforms[0].child_frame_id.size      = strlen(tf_base_buf);
  tf_transforms[0].child_frame_id.capacity  = sizeof(tf_base_buf);
#endif  // ODOM_ON_ESP32

  Serial.printf("Odometry tinh o: %s\n",
                ODOM_ON_ESP32 ? "ESP32 (phat /odom + /tf)"
                              : "LAPTOP (ESP32 chi phat /joint_states)");
  Serial.printf("LaserScan: %d bin, uoc luong %d byte/message (tran %d)\n",
                SCAN_BINS, EST_MSG_BYTES, MICROROS_MTU);
}

// ============================================================================
//  14. IN NGUYEN NHAN KHOI DONG LAI
// ============================================================================
static void print_reset_reason() {
  esp_reset_reason_t r = esp_reset_reason();
  Serial.println("\n======================================");
  switch (r) {
    case ESP_RST_POWERON:  Serial.println(">>> Reset: CAP NGUON"); break;
    case ESP_RST_BROWNOUT: Serial.println(">>> Reset: SUT AP (brownout)");
                           Serial.println("    -> Dong co an dong. Tach nguon dong co khoi nguon ESP32,");
                           Serial.println("       them tu 1000uF o 5V/GND."); break;
    case ESP_RST_PANIC:    Serial.println(">>> Reset: LOI CHUONG TRINH (panic)"); break;
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      Serial.println(">>> Reset: WATCHDOG (treo)"); break;
    case ESP_RST_SW:       Serial.println(">>> Reset: PHAN MEM"); break;
    case ESP_RST_EXT:      Serial.println(">>> Reset: NUT EN"); break;
    default:               Serial.printf( ">>> Reset: ma %d\n", (int)r); break;
  }
  Serial.println("======================================");
}

// ============================================================================
//  15. SETUP
// ============================================================================
void setup() {
  // --- Dua PWM ve 0 som nhat co the: tranh xe lao khi vua cap nguon ---
  pinMode(L_IN1_PIN, OUTPUT); pinMode(L_IN2_PIN, OUTPUT);
  pinMode(R_IN1_PIN, OUTPUT); pinMode(R_IN2_PIN, OUTPUT);
  // Hai lenh nay chiem kenh LEDC 0 va 1; LiDAR xin rieng kenh 2 o duoi nen khong dam.
  ledcAttach(L_ENA_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(R_ENB_PIN, PWM_FREQ, PWM_RES);

  wl.enc = &encL; wl.pin_in1 = L_IN1_PIN; wl.pin_in2 = L_IN2_PIN; wl.pin_pwm = L_ENA_PIN;
  wl.invert_motor = INVERT_MOTOR_L; wl.invert_enc = INVERT_ENC_L;
  wr.enc = &encR; wr.pin_in1 = R_IN1_PIN; wr.pin_in2 = R_IN2_PIN; wr.pin_pwm = R_ENB_PIN;
  wr.invert_motor = INVERT_MOTOR_R; wr.invert_enc = INVERT_ENC_R;

  // Ep trang thai ban dau: last_direction=0 nhung chan chua duoc dat -> ghi ro rang
  wl.last_direction = wr.last_direction = 99;   // khac 0 de lan goi dau tien thuc su ghi chan

  wheel_write_pwm(wl, 0);
  wheel_write_pwm(wr, 0);

  Serial.begin(115200);
  delay(300);
  print_reset_reason();

  // --- Encoder ---
  encL.attachHalfQuad(ENC_L_A_PIN, ENC_L_B_PIN);
  encR.attachHalfQuad(ENC_R_A_PIN, ENC_R_B_PIN);
  encL.setFilter(1023);
  encR.setFilter(1023);
  encL.clearCount();
  encR.clearCount();
  wl.prev_count = (long)encL.getCount();
  wr.prev_count = (long)encR.getCount();

  // --- BNO055 ---
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (bno.begin()) {
    bno.setExtCrystalUse(true);
    imu_ok = true;
    Serial.println("BNO055 san sang.");
  } else {
    imu_ok = false;
    Serial.println("CANH BAO: khong thay BNO055. Odometry se dung encoder cho goc quay.");
#if USE_GYRO_FOR_YAW
    Serial.println("          -> gyro_z = 0, xe se KHONG biet minh dang quay. Kiem tra day I2C!");
#endif
  }
  delay(500);

  // Phai goi TRUOC init_microros(): ham do cho Wi-Fi va agent, co the lau, va
  // luc nay moi chac chan xe dang dung yen. Dung xe trong 2 giay toi.
  calibrate_gyro_bias();

  // --- micro-ROS ---
  buf_mutex = xSemaphoreCreateMutex();
  clear_buffer(0);
  clear_buffer(1);
  init_microros();
  last_cmd_vel_ms = millis();

  // --- LiDAR ---
  Serial.print("LiDAR model "); Serial.println(lidar.getModelName());
  LidarSerial.setRxBufferSize(1024);        // phai goi TRUOC .begin()
  uint32_t baud = lidar.getSerialBaudRate();
  Serial.print("LiDAR baud "); Serial.println(baud);
  LidarSerial.begin(baud, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

  lidar.setScanPointCallback(lidar_scan_point_callback);
  lidar.setPacketCallback(lidar_packet_callback);
  lidar.setSerialWriteCallback(lidar_serial_write_callback);
  lidar.setSerialReadCallback(lidar_serial_read_callback);
  lidar.setMotorPinCallback(lidar_motor_pin_callback);
  lidar.setInfoCallback(lidar_info_callback);
  lidar.setErrorCallback(lidar_error_callback);
  lidar.init();
  LDS::result_t res = lidar.start();
  Serial.print("LiDAR start(): "); Serial.println(lidar.resultCodeToString(res));

  // --- Task ---
  // motor_task uu tien CAO HON ros_task: vong PID phai dung nhip 50 Hz,
  // khong duoc de viec gui goi tin lam tre.
  xTaskCreatePinnedToCore(motor_task, "MotorTask", 4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(ros_task,   "ROS_Task", 16384, NULL, 1, NULL, 0);
  // Core 1 con lai danh tron cho loop() -> lidar.loop()

  Serial.println("San sang. /cmd_vel -> banh xe | /scan /odom /tf /imu/data -> Nav2");
}

// ============================================================================
//  16. LOOP - Core 1, CHI doc UART LiDAR
// ============================================================================
void loop() {
  lidar.loop();     // tuyet doi khong them gi block vao day

  static uint32_t last_log = 0;
  if (millis() - last_log > 1000) {
    last_log = millis();
    float x, y, th;
    portENTER_CRITICAL(&odom_mux);
    x = odom_x; y = odom_y; th = odom_th;
    portEXIT_CRITICAL(&odom_mux);

    Serial.printf("%sscan %.1f Hz (%lu diem) | RPM L%6.1f/%6.1f R%6.1f/%6.1f | "
                  "pwm %4d/%4d | odom %.2f %.2f %.0f do | gyroZ %.2f | loi pub %lu\n",
                  emergency_stop ? "[ESTOP] " : "",
                  scan_freq_hz, (unsigned long)points_last_scan,
                  wl.current_rpm, wl.target_rpm, wr.current_rpm, wr.target_rpm,
                  wl.output_pwm, wr.output_pwm,
                  x, y, th * 180.0f / (float)M_PI,
                  gyro_z_rps, (unsigned long)publish_failed);

    // Dong DO LUONG - cho biet ros_task cham o khau nao:
    //   ros_task/s thap  -> vong lap bi block, xem cot "max us" nao lon nhat
    //   pub/s thap ma moi cot max deu nho -> nghen o tang truyen micro-ROS
    Serial.printf("      ros_task %lu vong/s | pub/s: scan %lu odom %lu imu %lu"
                  " | max us: spin %lu, imu %lu, odom %lu, scan %lu, sync %lu,"
                  " ca_vong %lu\n",
                  (unsigned long)ros_loops,
                  (unsigned long)pub_scan, (unsigned long)pub_odom,
                  (unsigned long)pub_imu,
                  (unsigned long)t_spin_max,
                  (unsigned long)t_imu_max, (unsigned long)t_pub_max,
                  (unsigned long)t_scan_max, (unsigned long)t_sync_max,
                  (unsigned long)t_loop_max);
    ros_loops = pub_scan = pub_odom = pub_imu = 0;
    t_imu_max = t_pub_max = t_scan_max = t_loop_max = t_spin_max = t_sync_max = 0;
  }
}

// ============================================================================
//  PHU LUC A - HIEU CHINH ODOMETRY  (lam MOT LAN, truoc khi chay Nav2)
// ============================================================================
// Day la buoc quyet dinh xe chay muot hay lang quang. Dung bo qua.
//
// A1. Kiem tra encoder
//     Nhac xe len, quay tay mot banh dung 10 vong. So "count" in ra phai doi
//     ~12000 (= 10 * ENC_PULSES_PER_REV). Neu khac, sua ENC_PULSES_PER_REV.
//     Neu count GIAM khi quay toi, dao INVERT_ENC_x.
//
// A2. Kiem tra chieu dong co
//     Gui:  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
//             "{linear: {x: 0.1}}"
//     Ca hai banh phai quay TOI. Banh nao lui thi dao INVERT_MOTOR_x.
//
// A3. Hieu chinh WHEEL_RADIUS_M  (sai so quang duong)
//     Danh dau vach xuat phat, cho xe chay thang 2.00 m roi do quang duong that:
//         WHEEL_RADIUS_M_moi = WHEEL_RADIUS_M_cu * (quang_duong_that / 2.00)
//     Lap lai den khi sai so < 2%.
//
// A4. Hieu chinh WHEEL_SEPARATION_M  (sai so goc quay)
//     CHI lam khi USE_GYRO_FOR_YAW = 0. Voi gyro thi buoc nay khong can.
//     Cho xe quay tai cho dung 10 vong, do goc lech so voi huong ban dau:
//         WHEEL_SEPARATION_M_moi = WHEEL_SEPARATION_M_cu * (3600 / goc_quay_that_do)
//
// A5. Kiem tra cuoi
//     Mo RViz, Fixed Frame = odom. Day xe quanh phong roi quay ve cho cu.
//     Diem odom phai tro ve gan goc toa do. Lech qua 10% thi quay lai A3.
//
// ============================================================================
//  PHU LUC B - TUNE PID
// ============================================================================
// Bo he so hien tai lay tu motor_1_wheel_l298n.ino - da la gia tri khoi dau
// cho L298N. Van nen do lai tren xe that vi tai va dien ap khac nhau.
//
// B1. Kf (feedforward) - lam TRUOC, quan trong nhat
//     Tam dat Kp = Ki = Kd = 0. Cho mot banh chay o vai muc PWM co dinh, ghi
//     lai RPM on dinh. Kf = PWM / RPM (lay trung binh). Voi Kf dung, xe da
//     chay gan dung toc do ma chua can PID.
//
// B2. deadbandPWM
//     Tang PWM tu 0 len tung nac, tim muc PWM nho nhat ma banh BAT DAU quay.
//     Do la deadbandPWM.
//
// B3. Kp - tang dan den khi RPM bam setpoint nhanh ma chua dao dong.
// B4. Ki - tang dan de triet sai so du (RPM on dinh nhung khong dung bang setpoint).
// B5. Kd - thuong de rat nho hoac 0. Encoder nhieu thi Kd lam moi thu te hon.
//
// Theo doi bang dong "RPM L xx/yy" tren Serial: so dau la do duoc, so sau la
// dat. Hai so phai bam nhau trong vong ~0.3 s.
//
// ============================================================================
//  PHU LUC C - CAC LOI HAY GAP
// ============================================================================
// * Reset lien tuc, ly do "SUT AP (brownout)"
//   Dong co keo dong lam sut nguon ESP32. PHAI tach nguon dong co khoi nguon
//   logic, noi chung GND, va them tu 1000uF sat chan nguon dong co.
//
// * Xe chay giat cuc, RPM dao dong
//   Kp qua cao, hoac lpfAlpha qua lon (nhieu encoder lot vao PID). Ha Kp,
//   giam lpfAlpha xuong 0.1.
//
// * RViz bao "message too old" / TF extrapolation
//   Dong ho ESP32 chua dong bo. Kiem tra log co dong "Da thay agent!" va
//   rmw_uros_sync_session tra ve OK.
//
// * Ban do SLAM bi truot, tuong bi nhan doi
//   Odometry sai -> quay lai Phu luc A. Hoac LIDAR_ANGLE_OFFSET_DEG sai ->
//   dat vat can truoc mui xe va chinh lai.
//
// * Xe van chay khi da tat Nav2
//   Kiem tra CMD_VEL_TIMEOUT_MS co hoat dong. Day la lop bao ve phan mem;
//   xe that VAN CAN mot nut dung khan cap cat nguon dong co bang tay.
// ============================================================================
