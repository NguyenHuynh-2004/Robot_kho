// ============================================================================
//  LAY MAU LAI MOT VONG QUET THO VE LUOI BIN DEU - BANG NOI SUY
// ============================================================================
//  Tach rieng thanh header, khong phu thuoc Arduino, de kiem duoc bang g++ tren
//  laptop: firmware/tests/test_scan_resample.cpp (nam NGOAI thu muc sketch de
//  Arduino khong bien dich no vao firmware).
//
//  VAN DE CUA CACH CU (gan diem vao giua bin, giu nguyen khoang cach):
//  LaserScan bat moi tia o mot phuong vi co dinh k*1.125 do, con diem tho cua
//  YDLIDAR o goc le bat ky va lech pha giua cac vong. Gan diem vao giua bin la
//  DOI GOC MA GIU KHOANG CACH. Voi tuong nam xien so voi tia - tuong xa hau
//  nhu luon xien - lam vay keo diem RA KHOI mat tuong, va vi pha moi vong mot
//  khac nen diem nhay ra nhay vao. Sai so ty le voi khoang cach: tu 2.55 m tro
//  di vuot mot o luoi 5 cm.
//  Do tren xe that (tools/check_scan_noise.py): nhieu DOC tia chi 3-10 mm toi
//  5 m, trong khi sai so do gan bin la 25-41 mm o 2.5-5 m. Tuc phan lon cai
//  "diem xa nhay loan" la do cach dong goi, khong phai do cam bien.
//
//  CACH MOI: voi moi bin, lay hai diem tho KEP hai ben tam bin va noi suy khoang
//  cach tai dung tam bin. Diem nam tren mat tuong thay vi bi keo lech.
//  KHONG noi suy khi hai diem chenh nhau nhieu (mep vat) hoac cach nhau qua xa
//  (lo hong) - khi do lay diem GAN NHAT ve khoang cach trong bin, y het cach cu,
//  de vat can khong bi thu nho lai.
// ============================================================================
#pragma once
#include <math.h>
#include <stdlib.h>

struct RawPt {
  float b;   // phuong vi trong he ROS, DO, [0, 360)
  float r;   // khoang cach, met, da loc theo [RANGE_MIN, RANGE_MAX]
};

static int raw_pt_cmp(const void *x, const void *y) {
  float a = ((const RawPt *)x)->b, c = ((const RawPt *)y)->b;
  return (a < c) ? -1 : (a > c ? 1 : 0);
}

// pts : n diem tho cua MOT vong (bi sap xep lai tai cho).
// out : nbins gia tri; bin k nam o phuong vi k*360/nbins. 0 = khong co du lieu.
static void resample_scan(RawPt *pts, int n, float *out, int nbins) {
  for (int k = 0; k < nbins; k++) out[k] = 0.0f;
  if (n <= 0) return;
  qsort(pts, n, sizeof(RawPt), raw_pt_cmp);

  const float step = 360.0f / (float)nbins;
  const float half = 0.5f * step;
  // Hai diem cach nhau qua 3 bin la co lo hong (be mat den, khong phan xa) ->
  // noi suy qua do la bia ra tuong khong co that.
  const float max_gap = 3.0f * step;

  int j = 0;                       // diem dau tien co b >= tam bin
  for (int k = 0; k < nbins; k++) {
    const float c = k * step;
    while (j < n && pts[j].b < c) j++;

    // Lang gieng hai ben, co vong qua 0/360 do
    const RawPt &p0 = (j > 0) ? pts[j - 1] : pts[n - 1];
    const RawPt &p1 = (j < n) ? pts[j] : pts[0];
    float b0 = p0.b, b1 = p1.b;
    if (j == 0) b0 -= 360.0f;
    if (j == n) b1 += 360.0f;
    const float gap = b1 - b0;

    float r = 0.0f;
    if (n >= 2 && gap > 0.0f && gap <= max_gap) {
      // Cung mot be mat khi chenh lech khoang cach nho: 5 cm hoac 5%, lay cai
      // lon hon. Lon hon nguong nay la mep vat -> khong noi suy.
      const float tol = fmaxf(0.05f, 0.05f * fminf(p0.r, p1.r));
      if (fabsf(p1.r - p0.r) <= tol) {
        const float t = (c - b0) / gap;
        r = p0.r + t * (p1.r - p0.r);
      }
    }
    if (r == 0.0f) {
      // Khong noi suy duoc: lay diem NAM TRONG bin nay, uu tien khoang cach
      // GAN hon - giu dung tinh chat an toan cua cach cu.
      const bool in0 = (c - b0) < half;
      const bool in1 = (b1 - c) < half;
      if (in0 && in1)  r = fminf(p0.r, p1.r);
      else if (in0)    r = p0.r;
      else if (in1)    r = p1.r;
    }
    out[k] = r;
  }
}
