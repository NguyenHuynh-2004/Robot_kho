#!/usr/bin/env python3
"""Ve anh xem truoc sa ban tren ban do - dung CHINH logic grid_navigator dung
(grid_board.py), nen diem/canh bi bo trong anh = diem/canh xe se tranh.

    cd ~/robot_ws_3 && python3 tools/preview_grid.py
    python3 tools/preview_grid.py --robot -0.55 0.04 178     # ve them xe
    python3 tools/preview_grid.py --sim                      # sa ban mo phong

Mac dinh: grid_real.yaml + real_room_map.yaml -> docs/sa_ban_5x5_preview.png.
Chi doc file, khong can ROS dang chay.
"""
import argparse
import math
import os
import sys

import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from grid_board import Board, CONFIG_DIR, config_path, edge_key   # noqa: E402

FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
FONT_B = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'


def load_map(path):
    meta = yaml.safe_load(open(path))
    img = os.path.join(os.path.dirname(path), meta['image'])
    px = np.array(Image.open(img).convert('L'))[::-1]       # hang 0 = y nho nhat
    data = np.full(px.shape, -1, np.int16)
    data[px > 250] = 0
    data[px < 50] = 100
    return data, meta['resolution'], meta['origin'][0], meta['origin'][1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sim', action='store_true', help='sa ban mo phong (grid_sim.yaml)')
    ap.add_argument('--grid', help='file sa ban khac')
    ap.add_argument('--map', help='file ban do .yaml khac')
    ap.add_argument('--robot', nargs=3, type=float, metavar=('X', 'Y', 'YAW_DO'),
                    help='ve them xe o (X, Y) quay YAW do')
    ap.add_argument('-o', '--out', help='file anh ra')
    a = ap.parse_args()

    real = not a.sim
    board = Board.load(a.grid or config_path(real))
    map_path = a.map or os.path.join(CONFIG_DIR, 'real_room_map.yaml' if real else 'my_room_map.yaml')
    out = a.out or os.path.join(ROOT, 'docs', f"sa_ban_{board.rows}x{board.cols}_preview"
                                f"{'' if real else '_mo_phong'}.png")
    data, res, ox, oy = load_map(map_path)
    H, W = data.shape
    bad_nodes, bad_edges, node_dist = board.check_map(data.ravel(), W, H, res, ox, oy)

    from scipy.ndimage import distance_transform_edt
    free = data == 0
    near = free & (distance_transform_edt(free) * res < board.node_clearance)

    # Nen ban do, phong to S lan
    S = max(2, int(round(0.3 / res)))           # 6 px moi o 5 cm
    rgb = np.zeros((H, W, 3), np.uint8)
    rgb[:] = (200, 200, 200)
    rgb[free] = (255, 255, 255)
    rgb[near] = (255, 228, 205)
    rgb[data == 100] = (0, 0, 0)
    base = Image.fromarray(rgb[::-1].repeat(S, 0).repeat(S, 1))
    L, T, R, B = 60, 60, 400, 50
    cv = Image.new('RGB', (base.width + L + R, base.height + T + B), 'white')
    cv.paste(base, (L, T))
    d = ImageDraw.Draw(cv)
    F = lambda s, b=False: ImageFont.truetype(FONT_B if b else FONT, s)
    INK, SUB = (30, 35, 45), (90, 95, 110)
    BLUE, LBLUE, RED = (0, 80, 220), (40, 150, 255), (230, 60, 60)

    def P(x, y):
        return L + (x - ox) / res * S, T + base.height - 1 - (y - oy) / res * S

    for na, nb in board.edges():
        bad = edge_key(na, nb) in bad_edges
        d.line([P(*board.xy(*na)), P(*board.xy(*nb))], fill=RED if bad else LBLUE, width=3)
    for r, c in board.nodes():
        u, q = P(*board.xy(r, c))
        d.rectangle([u - 7, q - 7, u + 7, q + 7], fill=RED if (r, c) in bad_nodes else BLUE)
    for c in range(board.cols):
        u, q = P(*board.xy(board.rows - 1, c))
        s = str(c + 1); tw = d.textlength(s, font=F(22, True))
        d.text((u - tw / 2, q - 42), s, fill=INK, font=F(22, True))
    for r in range(board.rows):
        u, q = P(*board.xy(r, 0))
        s = str(r + 1); tw = d.textlength(s, font=F(22, True))
        d.text((u - 26 - tw, q - 14), s, fill=INK, font=F(22, True))
    u, q = P(0, 0)
    d.line([u - 14, q, u + 14, q], fill=(0, 150, 0), width=3)
    d.line([u, q - 14, u, q + 14], fill=(0, 150, 0), width=3)
    if a.robot:
        rx, ry, ryaw = a.robot
        cx, cy = P(rx, ry)
        rr = board.robot_radius / res * S
        d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], fill=(255, 140, 0), outline=(150, 70, 0), width=2)
        ang = math.radians(ryaw); Lr = 0.45 / res * S
        ex, ey = cx + Lr * math.cos(ang), cy - Lr * math.sin(ang)
        d.line([cx, cy, ex, ey], fill=(150, 70, 0), width=5)
        for sg in (1, -1):
            b_ = ang + math.pi + sg * 0.45
            d.line([ex, ey, ex + 18 * math.cos(b_), ey - 18 * math.sin(b_)], fill=(150, 70, 0), width=5)

    x = L + base.width + 25; y = T
    n_all = board.rows * board.cols
    n_e = sum(1 for _ in board.edges())
    d.text((x, y), f"Sa bàn {board.rows}×{board.cols}, ô {board.spacing * 100:.0f} cm", fill=INK, font=F(24, True)); y += 40
    for line in (f"(hàng {board.center_row}, cột {board.center_col}) = "
                 f"({board.center_x:+.3f}, {board.center_y:+.3f}) m",
                 f"x {board.x0:+.2f} .. {board.x0 + (board.cols - 1) * board.spacing:+.2f} m,"
                 f"  y {board.y0:+.2f} .. {board.y0 + (board.rows - 1) * board.spacing:+.2f} m",
                 f"{n_all - len(bad_nodes)}/{n_all} điểm, {n_e - len(bad_edges)}/{n_e} cạnh đi được",
                 "hàng 1 = dưới cùng, cột 1 = bên trái",
                 f"file: config/{os.path.basename(board.path)}"):
        d.text((x, y), line, fill=INK, font=F(17)); y += 27
    for r, c in sorted(bad_nodes):
        d.text((x, y), f"Bỏ: hàng {r + 1}, cột {c + 1} (cách vật cản {node_dist[(r, c)] * 100:.0f} cm)",
               fill=RED, font=F(17)); y += 27
    y += 14
    leg = [((BLUE, 'sq'), "điểm xe tới được"), ((RED, 'sq'), f"điểm bỏ (< {board.node_clearance * 100:.0f} cm tới vật cản)"),
           ((LBLUE, 'ln'), "cạnh đi được"), ((RED, 'ln'), f"cạnh bỏ (< {board.edge_clearance * 100:.0f} cm)"),
           (((255, 228, 205), 'bx'), f"sát tường < {board.node_clearance * 100:.0f} cm"),
           (((0, 0, 0), 'bx'), "tường / vật cản"), (((0, 150, 0), 'x'), "gốc bản đồ (0, 0)")]
    if a.robot:
        leg.insert(0, (((255, 140, 0), 'ci'), "xe + hướng đầu xe"))
    for (col, kind), text in leg:
        if kind == 'sq':
            d.rectangle([x, y + 3, x + 12, y + 15], fill=col)
        elif kind == 'ln':
            d.line([x - 2, y + 9, x + 14, y + 9], fill=col, width=3)
        elif kind == 'bx':
            d.rectangle([x - 2, y + 1, x + 14, y + 17], fill=col, outline=(150, 150, 150))
        elif kind == 'ci':
            d.ellipse([x - 1, y, x + 15, y + 16], fill=col, outline=(150, 70, 0), width=2)
        else:
            d.line([x - 3, y + 9, x + 15, y + 9], fill=col, width=3)
            d.line([x + 6, y, x + 6, y + 18], fill=col, width=3)
        d.text((x + 26, y), text, fill=INK, font=F(17)); y += 30
    px1 = 1.0 / res * S
    y += 8
    d.line([x, y + 10, x + px1, y + 10], fill=INK, width=4)
    d.line([x, y + 3, x, y + 17], fill=INK, width=3)
    d.line([x + px1, y + 3, x + px1, y + 17], fill=INK, width=3)
    d.text((x + px1 / 2 - 14, y + 18), "1 m", fill=INK, font=F(17))
    cv.save(out)
    print(f"{n_all - len(bad_nodes)}/{n_all} diem, {n_e - len(bad_edges)}/{n_e} canh di duoc; "
          f"bo: {[(r + 1, c + 1) for r, c in sorted(bad_nodes)]}")
    print(f"anh: {out}")


if __name__ == '__main__':
    main()
