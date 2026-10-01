// ============================================================================
//  DO SO XUNG ENCODER TREN MOT VONG TRUC RA  (JGA25-370, encoder 2 kenh A/B)
// ============================================================================
//  Sketch rieng, KHONG micro-ROS, KHONG Wi-Fi, KHONG chay dong co. Chi doc
//  encoder va in ra Serial, de do ENC_PULSES_PER_REV cho esp32_agv_base.ino.
//
//  VI SAO PHAI DO:
//  So 1200 trong esp32_agv_base.ino chep tu sketch goc, chua ai kiem. Voi
//  JGA25-370 thi so dem tren mot vong TRUC RA bang:
//
//      xung_moi_vong_truc_ra = PPR_encoder * he_so_dem * ti_so_hop_so
//
//  Ca ba deu khac nhau giua cac lo hang (PPR thuong 11 hoac 13; hop so 1:9.6,
//  1:21, 1:34, 1:56...). Doan la sai, do thi chac.
//
//  he_so_dem o day la 2, vi dung attachHalfQuad GIONG HET firmware chinh. Doi
//  sang attachFullQuad la so dem gap doi - khi do ENC_PULSES_PER_REV cung phai
//  gap doi theo.
//
//  CACH DUNG
//      1. Nap sketch nay (giu nut BOOT luc Connecting...).
//      2. Mo Serial Monitor, dat 115200 baud, dong "Both NL & CR".
//      3. NHAC XE LEN cho banh quay tu do.
//      4. Danh dau mot diem tren LOP de dem vong cho chac.
//      5. Go  r  roi Enter  -> dua ca hai bo dem ve 0.
//      6. Quay TAY banh dung N vong theo chieu TOI (N cang lon cang chinh xac,
//         nen lay 10).
//      7. Go so vong vua quay, vi du  10  roi Enter.
//         -> no in ra so xung tren mot vong, chinh la ENC_PULSES_PER_REV.
//      8. Lam lai cho banh con lai de doi chieu hai ben.
//
//  Xong thi nap lai esp32_agv_base.ino.
// ============================================================================

#include <Arduino.h>
#include <ESP32Encoder.h>

// Giong het esp32_agv_base.ino. Doi o day thi phai doi ca ben kia.
const int ENC_L_A_PIN = 23;
const int ENC_L_B_PIN = 34;
const int ENC_R_A_PIN = 4;
const int ENC_R_B_PIN = 35;

// Gia tri dang nam trong esp32_agv_base.ino, de doi chieu voi so do duoc.
const float PPR_TRONG_FIRMWARE = 1200.0f;

ESP32Encoder encL, encR;
String dong;

static void in_huong_dan() {
  Serial.println();
  Serial.println("========================================================");
  Serial.println(" DO SO XUNG ENCODER TREN MOT VONG TRUC RA");
  Serial.println("========================================================");
  Serial.println(" r        -> dua hai bo dem ve 0");
  Serial.println(" <so>     -> bao da quay bao nhieu vong, vd: 10");
  Serial.println(" ?        -> in lai huong dan nay");
  Serial.println();
  Serial.println(" Nhac xe len, danh dau mot diem tren lop, go r, quay tay");
  Serial.println(" dung 10 vong theo chieu TOI, roi go 10 va Enter.");
  Serial.println("========================================================");
}

void setup() {
  Serial.begin(115200);
  delay(400);

  // GPIO 34/35 chi vao va khong co dien tro keo noi bo. Thu vien se in
  // "gpio_pullup_en(85): GPIO number error" luc khoi dong - VO HAI, bo qua.
  // Nhung neu encoder la loai ho cuc thu thi kenh B BAT BUOC phai co dien tro
  // 10k keo len 3V3 ben ngoai, khong thi so dem se loan.
  encL.attachHalfQuad(ENC_L_A_PIN, ENC_L_B_PIN);
  encR.attachHalfQuad(ENC_R_A_PIN, ENC_R_B_PIN);
  encL.setFilter(1023);
  encR.setFilter(1023);
  encL.clearCount();
  encR.clearCount();

  in_huong_dan();
}

void loop() {
  // --- In so dem song, 5 lan/giay ---
  static uint32_t lan_in = 0;
  if (millis() - lan_in > 200) {
    lan_in = millis();
    long cl = (long)encL.getCount();
    long cr = (long)encR.getCount();
    Serial.printf("TRAI %8ld   |   PHAI %8ld\n", cl, cr);
  }

  // --- Doc lenh tu Serial Monitor ---
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      dong.trim();
      if (dong.length() == 0) {
        // Enter khong -> bo qua
      } else if (dong == "r" || dong == "R") {
        encL.clearCount();
        encR.clearCount();
        Serial.println(">>> Da dua ve 0. Gio quay tay banh di.");
      } else if (dong == "?") {
        in_huong_dan();
      } else {
        float vong = dong.toFloat();
        if (vong <= 0.0f) {
          Serial.printf(">>> Khong hieu \"%s\". Go  r  hoac so vong, vd 10.\n",
                        dong.c_str());
        } else {
          long cl = (long)encL.getCount();
          long cr = (long)encR.getCount();
          Serial.println();
          Serial.println("--------------------------------------------------");
          Serial.printf("Da quay %.2f vong\n", vong);
          Serial.println("--------------------------------------------------");
          for (int i = 0; i < 2; i++) {
            const char *ten = i ? "PHAI" : "TRAI";
            long cnt = i ? cr : cl;
            if (labs(cnt) < 10) {
              Serial.printf("  %-5s %8ld xung  -> banh nay khong quay\n",
                            ten, cnt);
              continue;
            }
            float ppr = fabsf((float)cnt) / vong;
            Serial.printf("  %-5s %8ld xung  ->  %.1f xung/vong\n", ten, cnt, ppr);
            // Dau AM o mot ben la BINH THUONG: hai dong co lap doi xung nen khi
            // xe tien chung quay nguoc chieu nhau, va quan he pha A/B cho dau
            // nguoc nhau. Dung la hai co INVERT_ENC_x phai KHAC nhau.
            // Chi dang lo khi CA HAI cung dau luc quay theo chieu xe tien.
            Serial.printf("         dau %s -> INVERT_ENC_%c phai la %s\n",
                          cnt < 0 ? "AM " : "DUONG", ten[0] == 'T' ? 'L' : 'R',
                          cnt < 0 ? "true" : "false");
            Serial.printf("         lech %+.1f%% so voi %.0f dang dung trong firmware\n",
                          (ppr - PPR_TRONG_FIRMWARE) / PPR_TRONG_FIRMWARE * 100.0f,
                          PPR_TRONG_FIRMWARE);
          }
          Serial.println();
          Serial.println("  Lay trung binh hai banh, dat vao ENC_PULSES_PER_REV");
          Serial.println("  trong esp32_agv_base.ino. Khong can la so tron.");
          Serial.println("  Hai banh lech nhau qua vai % moi la van de: mot ben");
          Serial.println("  mat xung, thuong do kenh B tha noi tren GPIO 34/35.");
          Serial.println("--------------------------------------------------");
          encL.clearCount();
          encR.clearCount();
          Serial.println(">>> Da dua ve 0 cho lan do tiep theo.");
        }
      }
      dong = "";
    } else {
      dong += c;
      if (dong.length() > 16) dong = "";   // chong rac tren duong truyen
    }
  }
}
