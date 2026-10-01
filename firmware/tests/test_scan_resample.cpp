// Kiem scan_resample.h tren laptop, khong can ESP32.
//
//   g++ -O2 -I firmware/esp32_agv_base firmware/tests/test_scan_resample.cpp
//       -o /tmp/test_resample  &&  /tmp/test_resample
//
// Phong gia 8x6 m co them mot cai hop (de kiem mep vat). LiDAR gia ~260 diem
// mot vong, lech pha ngau nhien giua cac vong, nhieu doc tia 8 mm - dung bang
// so do duoc tren YDLIDAR X2 cua xe. So CACH CU (gan diem vao giua bin, giu
// diem gan nhat) voi CACH MOI (noi suy) bang khoang cach tu moi diem toi be mat
// THAT gan nhat.
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <vector>
#include "scan_resample.h"

static const int NBINS = 320;
static const float RMIN = 0.10f, RMAX = 8.0f;

struct Seg { float ax, ay, bx, by; };
static std::vector<Seg> segs = {
  {-4, -3, 4, -3}, {4, -3, 4, 3}, {4, 3, -4, 3}, {-4, 3, -4, -3},   // tuong
  {2.5f, -0.2f, 3.0f, -0.2f}, {3.0f, -0.2f, 3.0f, 0.3f},           // hop
  {3.0f, 0.3f, 2.5f, 0.3f},   {2.5f, 0.3f, 2.5f, -0.2f},
  // chan ban 2 cm, cach LiDAR ~1.5 m: noi suy KHONG duoc xoa no
  {-1.00f, -0.51f, -0.98f, -0.51f}, {-0.98f, -0.51f, -0.98f, -0.49f},
  {-0.98f, -0.49f, -1.00f, -0.49f}, {-1.00f, -0.49f, -1.00f, -0.51f},
};
static const float LEG_X = -0.99f, LEG_Y = -0.50f;
static const float OX = 0.5f, OY = -0.5f;     // vi tri LiDAR

static float cast(float deg) {
  float dx = cosf(deg * (float)M_PI / 180.0f), dy = sinf(deg * (float)M_PI / 180.0f);
  float best = 1e9f;
  for (auto &s : segs) {
    float ex = s.bx - s.ax, ey = s.by - s.ay;
    float den = dx * ey - dy * ex;
    if (fabsf(den) < 1e-9f) continue;
    float wx = s.ax - OX, wy = s.ay - OY;
    float t = (wx * ey - wy * ex) / den;
    float u = (wx * dy - wy * dx) / den;
    if (t > 0 && u >= 0 && u <= 1) best = std::min(best, t);
  }
  return best;
}

static float dist_to_surface(float px, float py) {
  float best = 1e9f;
  for (auto &s : segs) {
    float ex = s.bx - s.ax, ey = s.by - s.ay;
    float t = ((px - s.ax) * ex + (py - s.ay) * ey) / (ex * ex + ey * ey);
    t = std::max(0.0f, std::min(1.0f, t));
    float qx = s.ax + t * ex - px, qy = s.ay + t * ey - py;
    best = std::min(best, sqrtf(qx * qx + qy * qy));
  }
  return best;
}

// Ngau nhien tat dinh, de chay lai ra cung ket qua
static unsigned long long rs = 88172645463325252ULL;
static float urand() { rs ^= rs << 13; rs ^= rs >> 7; rs ^= rs << 17;
                       return (rs >> 11) * (1.0f / 9007199254740992.0f); }
static float nrand() { float u1 = std::max(urand(), 1e-12f), u2 = urand();
                       return sqrtf(-2 * logf(u1)) * cosf(2 * (float)M_PI * u2); }

// CACH CU, chep dung tu esp32_agv_base.ino truoc khi sua
static void old_bin(const std::vector<RawPt> &pts, float *out) {
  for (int k = 0; k < NBINS; k++) out[k] = 0;
  for (auto &p : pts) {
    int b = (int)lroundf(p.b * NBINS / 360.0f) % NBINS;
    if (b < 0) b += NBINS;
    if (out[b] == 0 || p.r < out[b]) out[b] = p.r;
  }
}

int main() {
  const int REVS = 60;
  const float buckets[] = {0, 1, 2, 3, 4, 6};
  const int NB = 5;
  std::vector<float> err_old[NB], err_new[NB];
  long cover_old = 0, cover_new = 0, phantom_old = 0, phantom_new = 0;
  long leg_raw = 0, leg_old = 0, leg_new = 0;

  for (int rev = 0; rev < REVS; rev++) {
    const float sp = 360.0f / 260.0f;
    float phase = urand() * sp;
    std::vector<RawPt> pts;
    for (float a = phase; a < 360.0f; a += sp) {
      float r = cast(a) + 0.008f * nrand();
      if (r > RMIN && r < RMAX) pts.push_back({a, r});
    }
    float o[NBINS], nw[NBINS];
    old_bin(pts, o);
    std::vector<RawPt> tmp = pts;
    resample_scan(tmp.data(), (int)tmp.size(), nw, NBINS);

    // Chan ban co bi bat duoc trong vong nay khong?
    bool raw_hit = false;
    for (auto &p : pts) {
      float c = p.b * (float)M_PI / 180.0f;
      if (hypotf(OX + p.r * cosf(c) - LEG_X, OY + p.r * sinf(c) - LEG_Y) < 0.03f)
        raw_hit = true;
    }
    if (raw_hit) leg_raw++;
    bool hit[2] = {false, false};

    for (int k = 0; k < NBINS; k++) {
      float c = k * 360.0f / NBINS * (float)M_PI / 180.0f;
      for (int m = 0; m < 2; m++) {
        float r = m ? nw[k] : o[k];
        if (r <= 0) continue;
        float px = OX + r * cosf(c), py = OY + r * sinf(c);
        if (hypotf(px - LEG_X, py - LEG_Y) < 0.04f) hit[m] = true;
        float e = dist_to_surface(px, py);
        (m ? cover_new : cover_old)++;
        if (e > 0.10f) (m ? phantom_new : phantom_old)++;
        for (int b = 0; b < NB; b++)
          if (r >= buckets[b] && r < buckets[b + 1]) (m ? err_new : err_old)[b].push_back(e);
      }
    }
    if (hit[0]) leg_old++;
    if (hit[1]) leg_new++;
  }

  auto pct = [](std::vector<float> v, float q) {
    if (v.empty()) return 0.0f;
    std::sort(v.begin(), v.end());
    return v[(size_t)(q * (v.size() - 1))];
  };
  printf("Do lech tu diem toi be mat THAT (mm), %d vong quet\n\n", REVS);
  printf("  khoang (m)   cu: trung vi    90%%   moi: trung vi    90%%\n");
  bool ok = true;
  for (int b = 0; b < NB; b++) {
    float om = pct(err_old[b], .5f) * 1000, o9 = pct(err_old[b], .9f) * 1000;
    float nm = pct(err_new[b], .5f) * 1000, n9 = pct(err_new[b], .9f) * 1000;
    printf("  %3.0f - %-3.0f   %10.1f %7.1f   %12.1f %7.1f\n",
           buckets[b], buckets[b + 1], om, o9, nm, n9);
    if (b >= 2 && !(n9 < o9)) ok = false;     // tu 2 m tro di phai tot hon
  }
  printf("\n  bin co du lieu moi vong : cu %.0f   moi %.0f\n",
         (float)cover_old / REVS, (float)cover_new / REVS);
  printf("  diem ma (lech > 10 cm)  : cu %ld   moi %ld\n", phantom_old, phantom_new);
  printf("  chan ban 2 cm @1.5 m    : diem tho cham %ld/%d vong, "
         "cu giu %ld, moi giu %ld\n", leg_raw, REVS, leg_old, leg_new);
  if (phantom_new > phantom_old) ok = false;   // khong duoc sinh them diem ma
  if (cover_new < cover_old) ok = false;       // khong duoc mat do phu
  if (leg_new < leg_old) ok = false;           // KHONG duoc xoa vat mong
  printf("\n%s\n", ok ? "DAT" : "CHUA DAT");
  return ok ? 0 : 1;
}
