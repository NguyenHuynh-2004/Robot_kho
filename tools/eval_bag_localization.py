#!/usr/bin/env python3
"""Do dinh vi tu mot bag ghi luc xe chay - TUNG anh quet, khong phai lay mau.

GHI BAG (trong luc lai xe, AMCL dang chay):
    ros2 bag record -o ~/bags/ten /scan /joint_states /imu/data /tf /tf_static         /odom /amcl_pose
DANH GIA:
    cd ~/robot_ws_3 && python3 tools/eval_bag_localization.py ~/bags/ten         src/my_bot_bringup/config/real_room_map.yaml

Voi moi anh quet (--every N: moi anh thu N):
  - tu the TF   = cai RViz hien (map -> lidar tai DUNG luc quet)
  - tu the THAT = tu the lam anh quet khop tuong nhat, tim quanh tu the TF
                  (+-40 cm, +-12 do, roi tinh chinh toi 1 cm / 0.25 do)
  - sai so dinh vi = TF - THAT, tach theo: dung yen / chay thang / quay tai cho
  - huong THAT tru huong cua tung nguon (odom, quaternion, gyro tich phan,
    banh xe) -> nguon goc quay nao troi

DOC KET QUA
  Sai vi tri luc chay: trung vi < 5 cm, p90 < 10 cm la anh quet bam tuong
  (o ban do 5 cm). "vuot cua so tim" nghia la lech > 40 cm / 12 do: lac han.
  Do chinh xac cua chinh phep do ~1-2 cm (o ban do 5 cm).

Chi doc bag, khong can ROS dang chay.
"""
import argparse
import bisect
import math
import sys

import numpy as np
import rosbag2_py
import yaml
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from scipy.ndimage import distance_transform_edt
import tf2_ros

ap = argparse.ArgumentParser()
ap.add_argument('bag')
ap.add_argument('map')
ap.add_argument('--every', type=int, default=3,
                help='danh gia moi anh quet thu N (1 = tat ca, cham hon)')
ap.add_argument('--csv')
ap.add_argument('--frame', default='map')
a = ap.parse_args()

TRUNC = 0.30


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(x):
    return (x + math.pi) % (2 * math.pi) - math.pi


# ---------------- ban do
meta = yaml.safe_load(open(a.map))
raw = open(a.map.rsplit('/', 1)[0] + '/' + meta['image'], 'rb').read()
tok, i = [], 0
while len(tok) < 4:
    while raw[i:i+1].isspace():
        i += 1
    if raw[i:i+1] == b'#':
        while raw[i:i+1] != b'\n':
            i += 1
        continue
    j = i
    while not raw[j:j+1].isspace():
        j += 1
    tok.append(raw[i:j]); i = j
i += 1
W, H = int(tok[1]), int(tok[2])
px = np.frombuffer(raw[i:i + W * H], dtype=np.uint8).reshape(H, W)[::-1]
RES = meta['resolution']
OX, OY = meta['origin'][0], meta['origin'][1]
wall = px < 50
DIST = distance_transform_edt(~wall) * RES


def dist_at(x, y, trunc=TRUNC):
    ix = np.floor((x - OX) / RES).astype(np.int64)
    iy = np.floor((y - OY) / RES).astype(np.int64)
    ins = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
    d = np.full(x.shape, trunc)
    d[ins] = np.minimum(DIST[iy[ins], ix[ins]], trunc)
    return d


# ---------------- doc bag
ro = rosbag2_py.SequentialReader()
ro.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id=''),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
types = {t.name: t.type for t in ro.get_all_topics_and_types()}
cls = {k: get_message(v) for k, v in types.items()}
buf = tf2_ros.Buffer(cache_time=Duration(seconds=1000))
scans, odom, imu, js, amcl = [], [], [], [], []
while ro.has_next():
    topic, data, _ = ro.read_next()
    m = deserialize_message(data, cls[topic])
    if topic == '/tf':
        for t in m.transforms:
            buf.set_transform(t, 'bag')
    elif topic == '/tf_static':
        for t in m.transforms:
            buf.set_transform_static(t, 'bag')
    elif topic == '/scan':
        scans.append(m)
    elif topic == '/odom':
        st = Time.from_msg(m.header.stamp).nanoseconds * 1e-9
        odom.append((st, m.twist.twist.linear.x, m.twist.twist.angular.z,
                     yaw_q(m.pose.pose.orientation)))
    elif topic == '/imu/data':
        q = m.orientation
        if abs(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z - 1) < 0.1:
            imu.append((Time.from_msg(m.header.stamp).nanoseconds * 1e-9,
                        m.angular_velocity.z, yaw_q(q)))
    elif topic == '/joint_states':
        try:
            il, ir = m.name.index('left_wheel_joint'), m.name.index('right_wheel_joint')
            js.append((Time.from_msg(m.header.stamp).nanoseconds * 1e-9,
                       m.position[il], m.position[ir], m.velocity[il], m.velocity[ir]))
        except (ValueError, IndexError):
            pass
    elif topic == '/amcl_pose':
        c = m.pose.covariance
        amcl.append((Time.from_msg(m.header.stamp).nanoseconds * 1e-9,
                     math.sqrt(max(c[0], c[7], 0)), math.degrees(math.sqrt(max(c[35], 0)))))
odom.sort(); imu.sort(); js.sort(); amcl.sort()
ot = [o[0] for o in odom]
t0 = Time.from_msg(scans[0].header.stamp).nanoseconds * 1e-9
print(f"{len(scans)} scan, {len(odom)} odom, {len(imu)} imu hop le, {len(js)} js, "
      f"{len(amcl)} amcl_pose, dai {Time.from_msg(scans[-1].header.stamp).nanoseconds*1e-9 - t0:.0f} s")

# ---------------- gyro tich phan (tru bias hoc khi dung yen) va goc banh
jt = [j[0] for j in js]


def wheel_vel(t):
    k = min(max(bisect.bisect_left(jt, t), 0), len(js) - 1)
    return js[k][3], js[k][4]


g_t, g_yaw = [], []
yaw, bias, win, prev = 0.0, 0.0, [], None
for t, gz, _ in imu:
    vl, vr = wheel_vel(t)
    if abs(vl) < 0.05 and abs(vr) < 0.05:
        win.append(gz)
        if len(win) >= 50:
            bias += 0.2 * (np.mean(win[-100:]) - bias)
    else:
        win = []
    if prev is not None and 0 < t - prev[0] < 0.5:
        yaw += (0.5 * (gz + prev[1]) - bias) * (t - prev[0])
    prev = (t, gz)
    g_t.append(t); g_yaw.append(yaw)
q_t = [x[0] for x in imu]
q_yaw = np.unwrap([x[2] for x in imu])
R_WHEEL, SEP = 0.03281, 0.200
w_yaw = [(j[2] - j[1]) * R_WHEEL / SEP for j in js]


def interp(ts, vs, t):
    return float(np.interp(t, ts, vs))


# ---------------- tung anh quet
dths1 = np.radians(np.arange(-12, 12.01, 1.5))
off1 = np.arange(-0.40, 0.4001, 0.04)
D1X, D1Y = [v.ravel() for v in np.meshgrid(off1, off1, indexing='ij')]
dths2 = np.radians(np.arange(-1.5, 1.501, 0.25))
off2 = np.arange(-0.04, 0.0401, 0.01)
D2X, D2Y = [v.ravel() for v in np.meshgrid(off2, off2, indexing='ij')]

rows = []
fails = 0
for k in range(0, len(scans), a.every):
    s = scans[k]
    t = Time.from_msg(s.header.stamp).nanoseconds * 1e-9
    try:
        tf = buf.lookup_transform(a.frame, s.header.frame_id, Time.from_msg(s.header.stamp))
        tob = buf.lookup_transform('odom', 'base_footprint', Time.from_msg(s.header.stamp))
    except Exception:
        fails += 1
        continue
    tx, ty = tf.transform.translation.x, tf.transform.translation.y
    th = yaw_q(tf.transform.rotation)
    r = np.array(s.ranges, dtype=np.float64)
    ang = s.angle_min + np.arange(len(r)) * s.angle_increment
    ok = (r > s.range_min) & (r < s.range_max) & np.isfinite(r)
    r, ang = r[ok], ang[ok]
    if len(r) < 60:
        continue
    d_tf = dist_at(tx + r * np.cos(ang + th), ty + r * np.sin(ang + th), 2.0)
    # tim tu the that
    sub = np.linspace(0, len(r) - 1, 90).astype(int)
    rs, angs = r[sub], ang[sub]
    best = (1e9, 0, 0, 0)
    for dth in dths1:
        c = dist_at(tx + D1X[:, None] + rs * np.cos(angs + th + dth),
                    ty + D1Y[:, None] + rs * np.sin(angs + th + dth)).mean(axis=1)
        j = int(np.argmin(c))
        if c[j] < best[0]:
            best = (c[j], D1X[j], D1Y[j], dth)
    b2 = best
    for dth in dths2:
        tt = th + best[3] + dth
        c = dist_at(tx + best[1] + D2X[:, None] + r * np.cos(ang + tt),
                    ty + best[2] + D2Y[:, None] + r * np.sin(ang + tt)).mean(axis=1)
        j = int(np.argmin(c))
        if c[j] < b2[0] or b2 is best:
            b2 = (c[j], best[1] + D2X[j], best[2] + D2Y[j], best[3] + dth)
    _, ex, ey, eth = b2
    bx, by, bth = tx + ex, ty + ey, th + eth
    d_best = dist_at(bx + r * np.cos(ang + bth), by + r * np.sin(ang + bth), 2.0)
    edge = (abs(best[1]) > 0.38 or abs(best[2]) > 0.38 or abs(best[3]) > math.radians(11.9))
    j = min(max(bisect.bisect_left(ot, t), 0), len(odom) - 1)
    v, w = odom[j][1], odom[j][2]
    ac = None
    if amcl:
        ka = bisect.bisect_right([x[0] for x in amcl], t) - 1
        if ka >= 0:
            ac = amcl[ka][1:]
    rows.append(dict(
        t=t - t0, v=v, w=w,
        fit_tf=float(np.median(d_tf)), hit_tf=float((d_tf < 0.10).mean()),
        fit_best=float(np.median(d_best)), hit_best=float((d_best < 0.10).mean()),
        epos=math.hypot(ex, ey), eth=math.degrees(eth), edge=edge,
        x=bx, y=by, yaw_true=bth, ex=ex, ey=ey,
        e_along=ex * math.cos(bth) + ey * math.sin(bth),
        e_across=-ex * math.sin(bth) + ey * math.cos(bth),
        ox=tob.transform.translation.x, oy=tob.transform.translation.y,
        yaw_odom=yaw_q(tob.transform.rotation),
        yaw_quat=interp(q_t, q_yaw, t), yaw_gyro=interp(g_t, g_yaw, t),
        yaw_wheel=interp(jt, w_yaw, t),
        amcl_pos=ac[0] if ac else float('nan'), amcl_deg=ac[1] if ac else float('nan')))
    if len(rows) % 100 == 0:
        print(f"  ... {len(rows)} scan", file=sys.stderr)

print(f"danh gia {len(rows)} scan (bo {fails} vi thieu TF)")
R = rows
good = [x for x in R if x['hit_best'] > 0.6]
print(f"scan co tu the that dang tin (>60% tia trung tuong): {len(good)}/{len(R)}")


def cls_of(x):
    v, w = abs(x['v']), abs(x['w'])
    if v < 0.02 and w < 0.05:
        return 'dung yen'
    if w < 0.15:
        return 'chay thang'
    if v < 0.05:
        return 'quay tai cho'
    return 'vua chay vua re'


def summ(xs, name):
    if not xs:
        print(f"  {name:16s}   -")
        return
    f = np.array([x['fit_tf'] for x in xs])
    p = np.array([x['epos'] for x in xs]) * 100
    h = np.abs([x['eth'] for x in xs])
    e = sum(x['edge'] for x in xs)
    print(f"  {name:16s} {len(xs):4d}  lech tuong {np.median(f)*100:5.1f} / "
          f"{np.percentile(f, 90)*100:5.1f} cm | sai vi tri {np.median(p):5.1f} / "
          f"{np.percentile(p, 90):5.1f} / max {p.max():5.1f} cm | sai huong "
          f"{np.median(h):4.1f} / {np.percentile(h, 90):4.1f} / max {h.max():4.1f} do"
          f"{f'  ({e} vuot cua so tim)' if e else ''}")


print("\n                    so   lech tuong (trung vi / p90)   sai vi tri (trung vi / p90 / max)   sai huong")
summ(good, 'TAT CA')
for c in ('dung yen', 'chay thang', 'quay tai cho', 'vua chay vua re'):
    summ([x for x in good if cls_of(x) == c], c)

print("\n--- theo thoi gian (moi 15 s) ---")
print(" t(s)   |w|max  v max  sai vitri  sai huong  amcl+-   lech tuong")
for t1 in range(0, int(R[-1]['t']) + 15, 15):
    xs = [x for x in good if t1 <= x['t'] < t1 + 15]
    if not xs:
        continue
    print(f"{t1:4d}   {max(abs(x['w']) for x in xs):5.2f}  {max(abs(x['v']) for x in xs):5.2f}"
          f"  {np.mean([x['epos'] for x in xs])*100:7.1f} cm {np.mean([abs(x['eth']) for x in xs]):7.1f} do"
          f"  {np.nanmean([x['amcl_pos'] for x in xs])*100:4.1f}cm"
          f"  {np.median([x['fit_tf'] for x in xs])*100:6.1f} cm")

# ---------------- nguon goc quay nao troi
print("\n--- huong THAT (tu khop anh quet) tru huong tung nguon, quy ve 0 o dau ---")
yt = np.unwrap([x['yaw_true'] for x in good])
for key, name in (('yaw_odom', 'odom (dang dung)'), ('yaw_quat', 'quaternion'),
                  ('yaw_gyro', 'gyro tich phan'), ('yaw_wheel', 'banh xe')):
    ys = np.unwrap([x[key] for x in good])
    d = np.degrees(yt - ys)
    d -= d[0]
    tot = np.degrees(np.sum(np.abs(np.diff(yt))))
    print(f"  {name:18s}: cuoi {d[-1]:+6.1f} do, dao dong {d.min():+6.1f} .. {d.max():+6.1f} "
          f"(bien do {d.max()-d.min():5.1f} do), do lech chuan {d.std():4.1f} do  "
          f"[xe da quay tong {tot:.0f} do]")

if a.csv:
    import csv
    with open(a.csv, 'w', newline='') as fh:
        wr = csv.DictWriter(fh, fieldnames=list(R[0].keys()))
        wr.writeheader()
        wr.writerows(R)
    print('csv:', a.csv)
