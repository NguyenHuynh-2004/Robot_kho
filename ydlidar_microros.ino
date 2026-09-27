// YDLIDAR X2/X2L tren ESP32 -> micro-ROS (Wi-Fi/UDP) -> sensor_msgs/LaserScan tren /scan
//
// Dua tren example cua thu vien LDS (KAIA.AI, Apache-2.0).
//
// Core 1 (loop)     : doc UART LiDAR, gom diem vao SCAN_BINS o goc deu nhau
// Core 0 (ros_task) : micro-ROS executor + publish LaserScan
//
// Laptop: ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

#ifndef ESP32
  #error This sketch runs on ESP32
#endif

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/laser_scan.h>

#include <LDS_YDLIDAR_X2_X2L.h>

// ==========================================================================
// 1. CAU HINH - SUA TRUOC KHI NAP
// ==========================================================================
#define WIFI_SSID   "SV BACH KHOA"
#define WIFI_PASS   "YOUR_WIFI_PASSWORD"
#define AGENT_IP    "192.168.1.21"   // IP laptop dang chay micro_ros_agent
#define AGENT_PORT  8888

#define ROBOT_ID    1                // doi neu chay nhieu ESP32 cung luc
#define SCAN_TOPIC  "scan"           // -> /scan
#define LASER_FRAME "laser"          // frame_id, phai khop Fixed Frame / TF ben RViz

// Huong quay: YDLIDAR quay CUNG chieu kim dong ho, ROS quy uoc goc tang NGUOC
// chieu kim dong ho -> mac dinh lat dau. Neu anh quet bi lat guong, doi thanh 0.
#define LIDAR_ANGLE_FLIP       1
// Bu lech goc gan LiDAR. Do thuc te: vat ben PHAI (270 do) hien o phia TRUOC
// (0 do) -> anh bi xoay, can bu 90 do. Cong thuc: a = 360 - (angle_deg + offset)
#define LIDAR_ANGLE_OFFSET_DEG 90.0f

// --- NGAN SACH KICH THUOC MESSAGE (quyet dinh ca toc do lan do phan giai) ---
// Thu vien micro_ros_arduino co MTU co dinh luc bien dich, nam trong
// src/uxr/client/config.h. Ban precompiled = 512 B; may nay da build lai
// thanh 1472 B bang laptop/rebuild_microros_mtu.sh.
//
//   Message <= MTU  -> BEST_EFFORT, 1 datagram, khong ACK  -> chay full 8.5 Hz
//   Message >  MTU  -> bat buoc RELIABLE + phan manh, ma ACK bam theo
//                        UXR_CONFIG_MIN_HEARTBEAT_TIME_INTERVAL = 100 ms
//                        -> tran thuc te chi ~3 Hz, RViz giat
//
//   kich thuoc ~= 80 + 4 * SCAN_BINS * (1 + PUBLISH_INTENSITIES)
//
// MICROROS_MTU phai khop dung voi thu vien dang cai:
//   512  = ban precompiled goc          -> toi da ~107 bin
//   1472 = sau khi chay rebuild_microros_mtu.sh -> toi da ~348 bin
// Dat sai gia tri o day se bi phat hien khi chay (LOI tang trong log Serial),
// chu trinh bien dich khong biet duoc.
#define MICROROS_MTU        1472
#define SCAN_BINS           320      // 1.125 do/tia (80 + 4*320 = 1360 B < 1472)
#define PUBLISH_INTENSITIES 0        // bat len se gap doi kich thuoc

#define EST_MSG_BYTES (80 + 4 * SCAN_BINS * (1 + PUBLISH_INTENSITIES))
#if EST_MSG_BYTES > MICROROS_MTU
  #error "Message vuot 1 datagram -> phai dung reliable va tut xuong ~3 Hz. Giam SCAN_BINS hoac nang MICROROS_MTU."
#endif

#define RANGE_MIN_M 0.10f
#define RANGE_MAX_M 8.00f

// --- Chan ket noi LiDAR ---
const uint8_t LIDAR_EN_PIN  = 19;
const uint8_t LIDAR_PWM_PIN = 15;
const uint8_t LIDAR_TX_PIN  = 33;
const uint8_t LIDAR_RX_PIN  = 32;

#define LIDAR_PWM_FREQ    10000
#define LIDAR_PWM_BITS    11
#define LIDAR_PWM_CHANNEL 2

// ==========================================================================
// 2. BIEN TOAN CUC
// ==========================================================================
HardwareSerial LidarSerial(2);
LDS_YDLIDAR_X2_X2L lidar;

rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
rclc_executor_t executor;
rcl_publisher_t scan_pub;
sensor_msgs__msg__LaserScan scan_msg;

static float scan_ranges[SCAN_BINS];   // bo dem cua message (chi ROS task cham vao)
static float scan_intens[SCAN_BINS];
static char  frame_id_buf[24] = LASER_FRAME;

// Ping-pong buffer chia se giua 2 core
static float bin_dist[2][SCAN_BINS];
static float bin_qual[2][SCAN_BINS];
static volatile uint8_t fill_idx   = 0;    // buffer dang duoc LiDAR ghi
static volatile uint8_t ready_idx  = 1;    // buffer da xong mot vong, cho publish
static volatile bool    scan_ready = false;
static volatile float   scan_freq_hz = 7.0f;

static SemaphoreHandle_t buf_mutex;

// Thong ke de in ra Serial
static volatile uint32_t scans_done = 0, scans_published = 0;
static volatile uint32_t points_in_scan = 0, points_last_scan = 0;
static volatile uint32_t publish_failed = 0;
static volatile int      last_pub_err = 0;

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { \
    Serial.printf("microROS error %d @ line %d\n", (int)rc, __LINE__); } }
#define RCSOFTCHECK(fn) { (void)(fn); }

// ==========================================================================
// 3. CALLBACK CUA THU VIEN LiDAR (chay tren Core 1)
// ==========================================================================
static void clear_buffer(uint8_t idx) {
  memset((void *)bin_dist[idx], 0, sizeof(bin_dist[idx]));
  memset((void *)bin_qual[idx], 0, sizeof(bin_qual[idx]));
}

int lidar_serial_read_callback() {
  return LidarSerial.read();
}

size_t lidar_serial_write_callback(const uint8_t *buffer, size_t length) {
  return LidarSerial.write(buffer, length);
}

void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality,
                               bool scan_completed) {
  // scan_completed == true nghia la diem NAY mo dau mot vong quet moi
  // -> dong goi vong cu lai va giao cho ROS task.
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
    points_last_scan = points_in_scan;   // tong diem hop le cua vong vua xong
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
  // Nhieu diem roi vao cung mot bin -> giu diem gan nhat (an toan cho tranh vat can)
  if (bin_dist[i][bin] == 0.0f || d_m < bin_dist[i][bin]) {
    bin_dist[i][bin] = d_m;
    bin_qual[i][bin] = quality;
  }
  points_in_scan++;
}

void lidar_packet_callback(uint8_t *packet, uint16_t length, bool scan_completed) {
  (void)packet; (void)length; (void)scan_completed;
}

void lidar_info_callback(LDS::info_t code, String info) {
  Serial.printf("LiDAR info %s: %s\n", lidar.infoCodeToString(code).c_str(), info.c_str());
}

void lidar_error_callback(LDS::result_t code, String aux_info) {
  Serial.printf("LiDAR error %s: %s\n", lidar.resultCodeToString(code).c_str(), aux_info.c_str());
}

void lidar_motor_pin_callback(float value, LDS::lds_pin_t lidar_pin) {
  int pin = (lidar_pin == LDS::LDS_MOTOR_EN_PIN) ? LIDAR_EN_PIN : LIDAR_PWM_PIN;

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

// ==========================================================================
// 4. MICRO-ROS (chay tren Core 0)
// ==========================================================================
static void fill_stamp() {
  if (rmw_uros_epoch_synchronized()) {
    int64_t t_ns = rmw_uros_epoch_nanos();
    scan_msg.header.stamp.sec     = (int32_t)(t_ns / 1000000000LL);
    scan_msg.header.stamp.nanosec = (uint32_t)(t_ns % 1000000000LL);
  } else {
    uint64_t t_ms = millis();
    scan_msg.header.stamp.sec     = (int32_t)(t_ms / 1000ULL);
    scan_msg.header.stamp.nanosec = (uint32_t)((t_ms % 1000ULL) * 1000000ULL);
  }
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

  fill_stamp();
  scan_msg.angle_min       = 0.0f;
  scan_msg.angle_increment = 2.0f * (float)M_PI / (float)SCAN_BINS;
  scan_msg.angle_max       = 2.0f * (float)M_PI - scan_msg.angle_increment;
  scan_msg.scan_time       = 1.0f / freq;
  scan_msg.time_increment  = scan_msg.scan_time / (float)SCAN_BINS;
  scan_msg.range_min       = RANGE_MIN_M;
  scan_msg.range_max       = RANGE_MAX_M;
  // range == 0 nam ngoai [range_min, range_max] -> RViz tu bo qua tia rong

  // PHAI kiem tra ma loi: neu message vuot buffer micro-ROS, rcl_publish that bai
  // ma khong co dau hieu gi khac - de lot la ngoi doan mo.
  rcl_ret_t rc = rcl_publish(&scan_pub, &scan_msg, NULL);
  if (rc == RCL_RET_OK) {
    scans_published++;
  } else {
    publish_failed++;
    last_pub_err = (int)rc;
  }
}

void ros_task(void *pvParameters) {
  (void)pvParameters;
  uint32_t last_sync = 0, last_ping = 0;
  int ping_fail = 0;

  for (;;) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
    publish_scan_if_ready();

    uint32_t now = millis();

    // Dong bo dong ho voi agent -> timestamp dung, RViz khong keu "message too old"
    if (now - last_sync > 10000) {
      last_sync = now;
      RCSOFTCHECK(rmw_uros_sync_session(200));
    }

    // Mat agent qua lau -> khoi dong lai cho sach
    if (now - last_ping > 2000) {
      last_ping = now;
      if (rmw_uros_ping_agent(150, 1) != RMW_RET_OK) {
        if (++ping_fail >= 5) {
          Serial.println("Mat ket noi agent -> restart ESP32");
          delay(100);
          ESP.restart();
        }
      } else {
        ping_fail = 0;
      }
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

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
  Serial.print("Wi-Fi OK, IP ESP32: ");
  Serial.println(WiFi.localIP());

  Serial.printf("Cho micro_ros_agent tai %s:%d", AGENT_IP, AGENT_PORT);
  while (rmw_uros_ping_agent(500, 1) != RMW_RET_OK) { Serial.print("."); delay(500); }
  Serial.println("\nDa thay agent!");

  char node_name[32];
  snprintf(node_name, sizeof(node_name), "esp32_lidar_node_%d", ROBOT_ID);

  allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
  rmw_uros_options_set_client_key(0x11111111 * ROBOT_ID, rmw_options);

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, node_name, "", &support));

  // BEST_EFFORT: chi hop le khi message <= 512 B (xem EST_MSG_BYTES o dau file).
  // Khong phan manh, khong cho ACK -> giu duoc dung nhip quay cua LiDAR.
  // Neu tang SCAN_BINS vuot 512 B thi phai doi sang rclc_publisher_init_default
  // (reliable) VA chinh RViz sang Reliable - nhung se tut xuong ~3 Hz.
  RCCHECK(rclc_publisher_init_best_effort(
    &scan_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
    SCAN_TOPIC));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rmw_uros_sync_session(1000));

  // Gan bo nho tinh cho message (micro-ROS khong tu cap phat sequence)
  sensor_msgs__msg__LaserScan__init(&scan_msg);
  scan_msg.header.frame_id.data     = frame_id_buf;
  scan_msg.header.frame_id.size     = strlen(frame_id_buf);
  scan_msg.header.frame_id.capacity = sizeof(frame_id_buf);
  scan_msg.ranges.data          = scan_ranges;
  scan_msg.ranges.size          = SCAN_BINS;
  scan_msg.ranges.capacity      = SCAN_BINS;
  scan_msg.intensities.data     = scan_intens;
  scan_msg.intensities.size     = PUBLISH_INTENSITIES ? SCAN_BINS : 0;
  scan_msg.intensities.capacity = SCAN_BINS;

  Serial.printf("LaserScan: %d bin, uoc luong %d byte/message (tran ~2048)\n",
                SCAN_BINS, EST_MSG_BYTES);
}

// ==========================================================================
// 5. SETUP / LOOP
// ==========================================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  buf_mutex = xSemaphoreCreateMutex();
  clear_buffer(0);
  clear_buffer(1);

  init_microros();

  // --- LiDAR ---
  Serial.print("LiDAR model ");
  Serial.println(lidar.getModelName());
  LidarSerial.setRxBufferSize(1024);          // phai goi TRUOC .begin()
  uint32_t baud_rate = lidar.getSerialBaudRate();
  Serial.print("LiDAR baud ");
  Serial.println(baud_rate);
  LidarSerial.begin(baud_rate, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

  lidar.setScanPointCallback(lidar_scan_point_callback);
  lidar.setPacketCallback(lidar_packet_callback);
  lidar.setSerialWriteCallback(lidar_serial_write_callback);
  lidar.setSerialReadCallback(lidar_serial_read_callback);
  lidar.setMotorPinCallback(lidar_motor_pin_callback);
  lidar.setInfoCallback(lidar_info_callback);   // example goc quen dang ky 2 dong nay
  lidar.setErrorCallback(lidar_error_callback);
  lidar.init();

  LDS::result_t result = lidar.start();
  Serial.print("LiDAR start(): ");
  Serial.println(lidar.resultCodeToString(result));
  if (result < 0) Serial.println("LiDAR da cam vao ESP32 chua?");

  // micro-ROS chay rieng tren Core 0, Core 1 danh toan bo cho UART LiDAR
  xTaskCreatePinnedToCore(ros_task, "ROS_Task", 16384, NULL, 1, NULL, 0);

  Serial.println("San sang. Publish sensor_msgs/LaserScan tren /scan");
}

void loop() {
  lidar.loop();   // chi doc/parse UART - tuyet doi khong block o day

  static uint32_t last_log = 0;
  if (millis() - last_log > 1000) {
    last_log = millis();
    Serial.printf("scan %.1f Hz | vong: %lu | diem/vong: %lu | publish OK: %lu | LOI: %lu (rc=%d)\n",
                  scan_freq_hz, (unsigned long)scans_done,
                  (unsigned long)points_last_scan, (unsigned long)scans_published,
                  (unsigned long)publish_failed, last_pub_err);
  }
}

// ==========================================================================
// GHI CHU: nang do phan giai
// ==========================================================================
// Tran kich thuoc message do thu vien micro_ros_arduino bien dich san quyet
// dinh, KHONG sua duoc tu sketch. Muon go tran, build lai thu vien:
//
//     bash ~/robot_ws_4/laptop/rebuild_microros_mtu.sh
//
// Script do dat UCLIENT_CUSTOM_TRANSPORT_MTU=1472 - datagram lon nhat khong bi
// phan manh o tang IP. Sau khi chay xong, quay len dau file sua 2 dong:
//
//     #define MICROROS_MTU  1472
//     #define SCAN_BINS     320      // 1.125 do/tia
//
// roi nap lai ESP32. Ket qua: 320 tia o full 8.5 Hz, van best-effort.
