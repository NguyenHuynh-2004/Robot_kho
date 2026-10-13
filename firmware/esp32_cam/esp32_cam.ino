// ESP32-CAM (AI-Thinker, OV2640) gan tren xe - chup anh hang cho cargo_task.py.
//
// Chon xe bang XE ben duoi: IP tinh 192.168.1.200 (xe 1) / .201 (xe 2).
// Wi-Fi lay tu wifi_config.h CUNG THU MUC (chep tu firmware/esp32_agv_base, chi
// can WIFI_SSID / WIFI_PASS; file nay khong vao git).
//
//   http://<ip>/capture     mot anh JPEG 640x480     (tools/esp32_cam_node.py mode capture)
//   http://<ip>:81/stream   luong MJPEG               (mode stream, xem bang trinh duyet)
//   http://<ip>/status      JSON: xe, rssi, up_s, so anh, reset
//
// Nap: Arduino IDE, board "AI Thinker ESP32-CAM", noi GPIO0 xuong GND khi nap,
// bo noi roi nhan RESET de chay. Nguon: 5 V RIENG >= 1 A (buck tu pin xe), tu
// 470 uF sat chan 5 V - ESP32-CAM keo dong dot bien khi phat Wi-Fi, nguon yeu
// thi khoi dong lai (BROWNOUT) giua chung.
#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include <WiFi.h>
#include "wifi_config.h"

#define XE 1                    // 1 hoac 2

// Camera lap nguoc (cap vang o tren) thi dat ca hai = 1
#define CAM_VFLIP   0
#define CAM_HMIRROR 0
#define JPEG_QUALITY 12         // 10-63, nho = net hon, anh to hon (~25-40 KB o VGA)

static const IPAddress CAM_IP(192, 168, 1, 199 + XE);
static const IPAddress GATEWAY(192, 168, 1, 1);
static const IPAddress SUBNET(255, 255, 255, 0);

// Chan AI-Thinker ESP32-CAM
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define LED_GPIO_NUM       4    // den flash - giu tat

static httpd_handle_t http80 = NULL, http81 = NULL;
static volatile uint32_t frames = 0;
static const char *reset_str = "?";

#define BOUNDARY "123456789000000000000987654321"

static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  esp_err_t r = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  frames++;
  return r;
}

static esp_err_t stream_handler(httpd_req_t *req) {
  char part[96];
  httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=" BOUNDARY);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return ESP_FAIL;
    size_t n = snprintf(part, sizeof(part),
                        "\r\n--" BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                        (unsigned)fb->len);
    esp_err_t r = httpd_resp_send_chunk(req, part, n);
    if (r == ESP_OK) r = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    frames++;
    if (r != ESP_OK) return r;          // trinh duyet / node dong ket noi
  }
}

static esp_err_t status_handler(httpd_req_t *req) {
  char buf[160];
  snprintf(buf, sizeof(buf),
           "{\"xe\":%d,\"ip\":\"%s\",\"rssi\":%d,\"up_s\":%lu,\"frames\":%lu,\"reset\":\"%s\",\"heap\":%lu}",
           XE, WiFi.localIP().toString().c_str(), (int)WiFi.RSSI(), (unsigned long)(millis() / 1000),
           (unsigned long)frames, reset_str, (unsigned long)ESP.getFreeHeap());
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req, buf);
}

static void start_servers() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80;
  cfg.ctrl_port = 32768;
  httpd_uri_t cap = {"/capture", HTTP_GET, capture_handler, NULL};
  httpd_uri_t st = {"/status", HTTP_GET, status_handler, NULL};
  httpd_uri_t root = {"/", HTTP_GET, status_handler, NULL};
  if (httpd_start(&http80, &cfg) == ESP_OK) {
    httpd_register_uri_handler(http80, &cap);
    httpd_register_uri_handler(http80, &st);
    httpd_register_uri_handler(http80, &root);
  }
  cfg.server_port = 81;
  cfg.ctrl_port = 32769;
  httpd_uri_t stream = {"/stream", HTTP_GET, stream_handler, NULL};
  if (httpd_start(&http81, &cfg) == ESP_OK) httpd_register_uri_handler(http81, &stream);
}

static bool init_camera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM; c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM; c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM; c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_VGA;           // 640x480 - cargo.yaml camera.width/height
  c.jpeg_quality = JPEG_QUALITY;
  c.fb_count = psramFound() ? 2 : 1;
  c.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;       // /capture luon tra anh moi nhat
  if (esp_camera_init(&c) != ESP_OK) return false;
  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, CAM_VFLIP);
  s->set_hmirror(s, CAM_HMIRROR);
  s->set_lenc(s, 1);                      // bu toi goc anh
  return true;
}

static void connect_wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.config(CAM_IP, GATEWAY, SUBNET, GATEWAY);
  WiFi.setSleep(false);                   // bo tiet kiem dien: anh khong bi tre
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Wi-Fi");
  for (int i = 0; i < 60 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(WiFi.status() == WL_CONNECTED ? " OK" : " CHUA DUOC (thu lai trong loop)");
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_GPIO_NUM, OUTPUT);
  digitalWrite(LED_GPIO_NUM, LOW);
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: reset_str = "POWERON"; break;
    case ESP_RST_BROWNOUT: reset_str = "BROWNOUT"; break;
    case ESP_RST_SW: reset_str = "SW"; break;
    case ESP_RST_PANIC: reset_str = "PANIC"; break;
    case ESP_RST_INT_WDT: case ESP_RST_TASK_WDT: case ESP_RST_WDT: reset_str = "WDT"; break;
    default: reset_str = "OTHER"; break;
  }
  Serial.printf("\nESP32-CAM xe %d, reset %s, PSRAM %s\n", XE, reset_str, psramFound() ? "co" : "KHONG");
  if (!init_camera()) {
    Serial.println("LOI: khong khoi tao duoc camera (cap FPC? nguon?) - khoi dong lai sau 5 s");
    delay(5000);
    ESP.restart();
  }
  connect_wifi();
  start_servers();
  Serial.printf("http://%s/capture   http://%s:81/stream\n",
                CAM_IP.toString().c_str(), CAM_IP.toString().c_str());
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last > 5000) {           // mat Wi-Fi thi noi lai
    last = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Mat Wi-Fi - noi lai");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
  }
  delay(100);
}
