#!/usr/bin/env python3
"""Dieu phoi NHIEU XE tren sa ban: giu cho truoc khi di, go ket chu dong.

Moi xe chay fleet_agent.py (namespace rieng). Node nay lap duong cho tung xe,
cho phep tung xe di toi o nao (giu cho), phat hien va go ket.

Quy tac:
 1. Hai xe khong cung dich; khong nhan dich la o xe khac dang do.
 2. Lap duong: it canh nhat, it re (moi lan re = 0.3 canh), cong them chi phi
    khi di tren duong con lai cua xe kia (2/o) va di nguoc chieu xe kia tren
    cung canh (6/canh). Xe dang do (khong co lo trinh) la vat can cung.
 3. Giu cho: xe CHIEM moi o cach no duoi nua o + ban kinh xe (dang chay giua
    hai o thi chiem ca hai, qua khoi nua o moi nha o sau). Xe duoc giu truoc
    toi LOOKAHEAD o tren lo trinh va chi duoc di toi o da giu. O da giu khong
    bi lay lai.
 4. Hai xe cung xin mot o: xe GAN DICH HON (it o con lai) duoc truoc; bang
    nhau thi xe ten nho hon.
 5. Cho > 3 s vi bi xe kia chan:
      - xe kia dang do: lap duong vong quanh no; khong duoc thi xe dang do ne
        sang o trong roi quay ve cho cu;
      - xe kia cung dang cho minh (ket): xe XA DICH HON nhuong - lap duong vong
        tranh xe kia; khong duoc thi ne sang o trong, doi xe kia qua roi di
        tiep; khong duoc nua thi doi vai; van khong duoc thi dung ca hai, bao loi;
      - xe kia dang chay / dang xoay: cho tiep; dung yen han > 3 s thi go nhu tren.
 6. Xe mat trang thai > 1 s: giu nguyen o no chiem (xe khac khong vao), bao
    loi. Xe bao vat can / Nav2 that bai: lap lai duong tu cho no dung.

    python3 traffic_manager.py agv1 agv2 [--grid FILE] [--no-avoid]
Nhap: "1 5 5 d" = xe 1 toi hang 5 cot 5 roi quay Dong; "2 1 1" (giu huong);
      "s" dung tat ca; "q" thoat. Hoac gui chuoi do vao topic /fleet/goal.
--no-avoid: bo chi phi tranh duong xe kia (de thu go ket).
"""
import heapq
import json
import math
import sys
import threading
import time
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from grid_board import Board, config_path, edge_key
from grid_navigator import HEADING_NAMES, parse_heading

TICK = 0.2                  # s
LOOKAHEAD = 2               # o giu truoc
WAIT_S = 3.0                # cho lau hon -> go ket
STALE_S = 1.0               # khong co trang thai lau hon -> mat lien lac
REACH_R = 0.15              # m - coi nhu da toi o
TURN_COST = 0.3
SOFT_NODE = 2.0
SOFT_GOAL = 3.0
SOFT_OPP = 6.0
RETRY_S = 2.0
MAX_FAIL = 5
COLORS = [(0.15, 0.45, 0.95), (0.95, 0.5, 0.1), (0.2, 0.75, 0.3), (0.8, 0.2, 0.7)]
DIRS = [(0, 1), (1, 0), (0, -1), (-1, 0)]       # Dong, Bac, Tay, Nam (theo truc sa ban)


class Robot:
    def __init__(self, name, idx):
        self.name, self.idx = name, idx
        self.x = self.y = self.yaw = None
        self.t = 0.0
        self.state = 'unknown'
        self.ack = None                 # route_id agent dang chay
        self.dyn_nodes, self.dyn_edges = set(), set()
        self.goal = None                # dich nguoi dung dat (r, c)
        self.goal_heading = None
        self.route = None               # [(r, c)] hoac None
        self.route_id = 0
        self.upto = 0
        self.progress = 0
        self.final_heading = None
        self.mode = 'idle'              # idle | go | yield | yield_wait
        self.resume = None              # (dich, huong) sau khi ne
        self.wait_since = None
        self.yield_since = None
        self.stale = False
        self.fail_count = 0
        self.retry_at = 0.0
        self.pending = False            # co dich nhung chua co vi tri xe
        self.mark = (None, None, None)  # vi tri lan cuoi thay xe nhuc nhich
        self.last_move_t = 0.0


class TrafficManager(Node):
    def __init__(self, names, grid_file=None, avoid=True):
        super().__init__('traffic_manager')
        self.board = Board.load(grid_file or config_path(True))
        self.robots = {n: Robot(n, i) for i, n in enumerate(names)}
        self.order = list(names)
        self.avoid = avoid
        self.occ_r = self.board.spacing / 2 + self.board.robot_radius
        self.static_nodes, self.static_edges = set(), set()
        self.map_ok = False
        self.next_route_id = 1
        self.cmd_pubs = {n: self.create_publisher(String, f'/fleet/{n}/cmd', 10) for n in names}
        self.goal_pub = self.create_publisher(String, '/fleet/goal', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/fleet/markers', 10)
        self.create_subscription(String, '/fleet/status', self.cb_status, 50)
        self.create_subscription(String, '/fleet/goal', self.cb_goal, 10)
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.cb_map, qos)
        self.create_timer(TICK, self.tick)
        self.occ, self.res = {}, {}
        self.get_logger().info(
            f"Dieu phoi {', '.join(names)} tren sa ban {self.board.rows}x{self.board.cols}"
            f"{'' if avoid else ' (TAT tranh duong xe kia)'}. Cho /map va trang thai xe...")

    # ------------------------------------------------------------------ tien ich
    def log(self, text, level='info'):
        # rclpy cam mot cho goi log doi muc giua cac lan -> moi muc mot cho goi
        lg = self.get_logger()
        if level == 'error':
            lg.error(text)
        elif level == 'warn':
            lg.warn(text)
        else:
            lg.info(text)

    @staticmethod
    def lbl(n):
        return f'({n[0] + 1},{n[1] + 1})'

    def route_str(self, nodes):
        return ' -> '.join(self.lbl(n) for n in nodes)

    def xy(self, n):
        return self.board.xy(*n)

    def dist(self, rb, n):
        if rb.x is None:
            return float('inf')
        x, y = self.xy(n)
        return math.hypot(rb.x - x, rb.y - y)

    def node_of(self, rb):
        """O gan xe nhat ma khong bi chan tinh."""
        cands = [n for n in self.board.nodes() if n not in self.static_nodes]
        return min(cands, key=lambda n: self.dist(rb, n))

    def occupied(self, rb):
        if rb.x is None:
            return set()
        return {n for n in self.board.nodes() if self.dist(rb, n) < self.occ_r}

    def remaining(self, rb):
        return rb.route[rb.progress:] if rb.route else []

    def prio(self, rb):
        """Nho hon = uu tien hon: it o con lai (gan dich hon), roi ten."""
        left = len(rb.route) - 1 - rb.progress if rb.route else 0
        return (left, rb.idx)

    def parked(self, rb):
        return rb.route is None

    def others(self, rb):
        return [o for o in self.robots.values() if o is not rb]

    def blocked_sets(self):
        nodes = set(self.static_nodes)
        edges = set(self.static_edges)
        for rb in self.robots.values():
            nodes |= rb.dyn_nodes
            edges |= rb.dyn_edges
        return nodes, edges

    def heading_dir(self, yaw):
        if yaw is None:
            return None
        d = math.atan2(math.sin(yaw - self.board.yaw), math.cos(yaw - self.board.yaw))
        return DIRS[int(round(d / (math.pi / 2))) % 4]

    # ------------------------------------------------------------------ lap duong
    def plan(self, me, start, goal, hard=frozenset()):
        """Dijkstra tren (o, huong vua di). None neu khong co duong."""
        bn, be = self.blocked_sets()
        bn = (bn | set(hard)) - {start}
        if goal in bn:
            return None
        soft_nodes, opp_edges, soft_goals = set(), set(), set()
        if self.avoid:
            for o in self.others(me):
                rem = self.remaining(o)
                soft_nodes |= set(rem)
                for a, b in zip(rem, rem[1:]):
                    opp_edges.add((b, a))           # minh di b->a = nguoc chieu xe kia
                if o.goal is not None:
                    soft_goals.add(o.goal)
        d0 = self.heading_dir(me.yaw) if me.x is not None and self.dist(me, start) < 0.2 else None
        best = {(start, d0): 0.0}
        prev = {}
        heap = [(0.0, 0, start, d0)]
        seq = 0
        while heap:
            cost, _, n, d = heapq.heappop(heap)
            if best.get((n, d), float('inf')) < cost - 1e-9:
                continue
            if n == goal:
                path = [n]
                key = (n, d)
                while key in prev:
                    key = prev[key]
                    path.append(key[0])
                return path[::-1]
            for nb in self.board.neighbours(*n):
                if nb in bn or edge_key(n, nb) in be:
                    continue
                nd = (nb[0] - n[0], nb[1] - n[1])
                c = cost + 1.0
                if d is not None and nd != d:
                    c += TURN_COST * (2 if nd == (-d[0], -d[1]) else 1)
                if nb in soft_nodes:
                    c += SOFT_NODE
                if nb in soft_goals and nb != goal:
                    c += SOFT_GOAL
                if (n, nb) in opp_edges:
                    c += SOFT_OPP
                key = (nb, nd)
                if c < best.get(key, float('inf')) - 1e-9:
                    best[key] = c
                    prev[key] = (n, d)
                    seq += 1
                    heapq.heappush(heap, (c, seq, nb, nd))
        return None

    def assign_route(self, rb, nodes, final_heading, mode):
        rb.route = list(nodes)
        rb.route_id = self.next_route_id
        self.next_route_id += 1
        rb.upto = 0
        rb.progress = 0
        rb.final_heading = final_heading
        rb.mode = mode
        rb.wait_since = None
        what = {'go': 'lo trinh', 'yield': 'NE DUONG'}.get(mode, mode)
        self.log(f'[{rb.name}] {what} #{rb.route_id}: {self.route_str(nodes)}')

    def clear_route(self, rb, reason=None):
        rb.route = None
        rb.route_id = self.next_route_id      # id moi -> agent dung xe
        self.next_route_id += 1
        rb.mode = 'idle'
        rb.resume = None
        rb.wait_since = None
        if reason:
            self.log(f'[{rb.name}] dung: {reason}', 'warn')

    # ------------------------------------------------------------------ callbacks
    def cb_map(self, msg):
        if self.map_ok:
            return
        i = msg.info
        self.static_nodes, self.static_edges, _ = self.board.check_map(
            msg.data, i.width, i.height, i.resolution, i.origin.position.x, i.origin.position.y)
        self.map_ok = True
        self.log(f'Ban do: bo {len(self.static_nodes)} o, {len(self.static_edges)} canh sat vat can')

    def cb_status(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        rb = self.robots.get(d.get('name'))
        if rb is None:
            return
        rb.t = time.time()
        if d.get('x') is not None:
            rb.x, rb.y, rb.yaw = d['x'], d['y'], d['yaw']
            mx, my, myaw = rb.mark
            if (mx is None or math.hypot(rb.x - mx, rb.y - my) > 0.02 or
                    abs(math.atan2(math.sin(rb.yaw - myaw), math.cos(rb.yaw - myaw))) > 0.05):
                rb.mark = (rb.x, rb.y, rb.yaw)
                rb.last_move_t = rb.t
        rb.state = d.get('state', 'unknown')
        rb.ack = d.get('route_id')
        rb.dyn_nodes = {tuple(n) for n in d.get('blocked_nodes', [])}
        rb.dyn_edges = {edge_key(tuple(a), tuple(b)) for a, b in d.get('blocked_edges', [])}
        if rb.stale:
            rb.stale = False
            self.log(f'[{rb.name}] co lai lien lac')

    def cb_goal(self, msg):
        line = msg.data.strip()
        if not line:
            return
        tok = line.split()
        if tok[0] in ('s', 'S', 'stop'):
            for rb in self.robots.values():
                rb.goal = None
                self.clear_route(rb, 'lenh dung tat ca')
            return
        name = tok[0]
        if name.isdigit() and 1 <= int(name) <= len(self.order):
            name = self.order[int(name) - 1]
        rb = self.robots.get(name)
        try:
            r, c = int(tok[1]) - 1, int(tok[2]) - 1
            heading = None
            if len(tok) > 3:
                _, heading = parse_heading(tok[3], self.board.yaw)
        except (IndexError, ValueError) as e:
            self.log(f'Lenh "{line}" sai ({e}). Dang: "1 5 5 d"', 'error')
            return
        if rb is None:
            self.log(f'Khong co xe "{tok[0]}"', 'error')
            return
        goal = (r, c)
        if not (0 <= r < self.board.rows and 0 <= c < self.board.cols):
            self.log(f'[{rb.name}] o {self.lbl(goal)} ngoai sa ban', 'error')
            return
        if goal in self.static_nodes:
            self.log(f'[{rb.name}] o {self.lbl(goal)} sat vat can, khong toi duoc', 'error')
            return
        for o in self.others(rb):
            # O xe kia DANG DO thi duoc: khi can, xe do ne sang o trong va o
            # lai do (vd hai xe doi cho). Chi cam trung dich / cho xe kia se ve.
            taken = {o.goal} if o.goal else set()
            if o.resume:
                taken.add(o.resume[0])
            if goal in taken:
                self.log(f'[{rb.name}] o {self.lbl(goal)} la dich cua {o.name}', 'error')
                return
        rb.goal, rb.goal_heading = goal, heading
        rb.resume = None
        rb.fail_count = 0
        if rb.x is None or not self.map_ok:
            rb.pending = True
            self.log(f'[{rb.name}] chua co {"vi tri xe" if rb.x is None else "/map"} - '
                     'se lap duong khi co')
            return
        self.route_to_goal(rb)

    def route_to_goal(self, rb):
        hard = set()
        for o in self.others(rb):
            if self.parked(o):
                hard |= self.occupied(o)
        start = self.node_of(rb)
        path = self.plan(rb, start, rb.goal, hard=hard)
        if path is None:
            path = self.plan(rb, start, rb.goal)     # vong khong duoc: di, toi do go ket
        if path is None:
            self.clear_route(rb, f'khong co duong toi {self.lbl(rb.goal)}')
            rb.goal = None
            return False
        self.assign_route(rb, path, rb.goal_heading, 'go')
        # Dich la cho xe khac dang do: bao xe do ne ngay, khoi doi toi sat roi
        # cho 3 s moi go ket
        for o in self.others(rb):
            if self.parked(o) and o.x is not None and self.node_of(o) == rb.goal:
                self.log(f'[{o.name}] dang do o dich cua {rb.name} -> ne truoc')
                self.occ[o.name] = self.occupied(o)
                self.occ[rb.name] = self.occupied(rb)
                if not self.side_step(o, rb):
                    self.log(f'[{o.name}] khong co o trong de ne', 'warn')
        return True

    # ------------------------------------------------------------------ vong chinh
    def tick(self):
        now = time.time()
        for rb in self.robots.values():
            if rb.t and now - rb.t > STALE_S and not rb.stale:
                rb.stale = True
                self.log(f'[{rb.name}] MAT LIEN LAC {now - rb.t:.1f} s - giu nguyen o xe chiem, '
                         'xe khac khong vao', 'error')
        self.occ = {n: self.occupied(rb) for n, rb in self.robots.items()}
        for rb in self.robots.values():
            if rb.pending and rb.x is not None and self.map_ok and rb.goal is not None:
                rb.pending = False
                self.route_to_goal(rb)

        # 1. Tien do, xong lo trinh, su co
        for rb in self.robots.values():
            if not rb.route or rb.ack != rb.route_id:
                continue
            for i in range(rb.progress, rb.upto + 1):
                if self.dist(rb, rb.route[i]) < REACH_R:
                    rb.progress = i
            if rb.state == 'done':
                self.on_route_done(rb, now)
            elif rb.state in ('blocked', 'failed'):
                self.on_trouble(rb, now)

        # 2. O da giu cua tung xe
        self.res = {}
        for n, rb in self.robots.items():
            s = set(self.occ[n])
            if rb.route:
                s |= set(rb.route[rb.progress:rb.upto + 1])
            self.res[n] = s

        # 3. Cap them o, xe gan dich truoc
        movers = [rb for rb in self.robots.values()
                  if rb.route and rb.ack == rb.route_id and not rb.stale
                  and rb.state not in ('blocked', 'failed')]
        for rb in sorted(movers, key=self.prio):
            want = min(rb.progress + LOOKAHEAD, len(rb.route) - 1)
            while rb.upto < want:
                nxt = rb.route[rb.upto + 1]
                if self.owner(nxt, rb) is not None:
                    break
                rb.upto += 1
                self.res[rb.name].add(nxt)

        # 4. Cho / ket
        for rb in movers:
            if rb.route is None or rb.upto >= len(rb.route) - 1 or rb.state != 'waiting':
                rb.wait_since = None
                continue
            blk = self.owner(rb.route[rb.upto + 1], rb)
            if blk is None:
                rb.wait_since = None
                continue
            if rb.wait_since is None:
                rb.wait_since = now
                self.log(f'[{rb.name}] cho o {self.lbl(rb.route[rb.upto + 1])} ({blk.name} dang giu)')
                continue
            # Xe chan dang chay / dang xoay thi cu cho (xe that xoay ~5 s moi
            # lan). Chi go khi no do, mat lien lac, cho nguoc lai minh, hoac
            # dung yen han > WAIT_S.
            stuck = (self.parked(blk) or blk.stale or self.waits_for(blk, rb)
                     or now - blk.last_move_t > WAIT_S)
            if stuck and now - rb.wait_since > WAIT_S:
                rb.wait_since = None
                self.resolve(rb, blk)

        # 5. Xe da ne: quay lai khi xe kia di qua
        for rb in self.robots.values():
            if rb.mode == 'yield_wait':
                self.try_resume(rb, now)

        # 6. Gui lenh
        for n, rb in self.robots.items():
            msg = String()
            msg.data = json.dumps({'route_id': rb.route_id, 'nodes': rb.route or [],
                                   'upto': rb.upto, 'final_heading': rb.final_heading})
            self.cmd_pubs[n].publish(msg)
        self.publish_markers()

    def owner(self, node, me):
        for n, s in self.res.items():
            if n != me.name and node in s:
                return self.robots[n]
        return None

    def waits_for(self, a, b):
        """a dang dung cho o ma b giu?"""
        if not a.route or a.state != 'waiting' or a.upto >= len(a.route) - 1:
            return False
        return a.route[a.upto + 1] in self.res.get(b.name, set())

    # ------------------------------------------------------------------ su kien
    def on_route_done(self, rb, now):
        if rb.mode == 'yield':
            if rb.resume is None:
                self.log(f'[{rb.name}] da ne sang {self.lbl(rb.route[-1])}, do lai day')
                rb.route = None
                rb.mode = 'idle'
                return
            self.log(f'[{rb.name}] da ne sang {self.lbl(rb.route[-1])}, cho xe kia qua')
            rb.route = None
            rb.mode = 'yield_wait'
            rb.yield_since = now
            return
        if rb.mode == 'go':
            g = rb.route[-1]
            hd = '' if rb.final_heading is None else f', huong {self.heading_label(rb.final_heading)}'
            self.log(f'[{rb.name}] TOI DICH {self.lbl(g)}{hd}')
        rb.route = None
        rb.mode = 'idle'
        rb.goal = None
        rb.fail_count = 0

    def heading_label(self, yaw):
        d = self.heading_dir(yaw)
        return {(0, 1): HEADING_NAMES['dong'], (1, 0): HEADING_NAMES['bac'],
                (0, -1): HEADING_NAMES['tay'], (-1, 0): HEADING_NAMES['nam']}[d]

    def on_trouble(self, rb, now):
        if now < rb.retry_at:
            return
        rb.retry_at = now + RETRY_S
        rb.fail_count += 1
        target, heading, mode = rb.route[-1], rb.final_heading, rb.mode
        if rb.fail_count > MAX_FAIL:
            rb.goal = None
            self.clear_route(rb, f'{rb.state} {rb.fail_count - 1} lan, bo lo trinh - can nguoi kiem tra')
            return
        hard = set()
        for o in self.others(rb):
            hard |= self.res.get(o.name, self.occupied(o))
        path = self.plan(rb, self.node_of(rb), target, hard=hard)
        if path is None:
            self.log(f'[{rb.name}] {rb.state}: chua co duong toi {self.lbl(target)}, thu lai sau '
                     f'{RETRY_S:.0f} s ({rb.fail_count}/{MAX_FAIL})', 'warn')
            return
        self.log(f'[{rb.name}] {rb.state} - lap lai duong ({rb.fail_count}/{MAX_FAIL})', 'warn')
        self.assign_route(rb, path, heading, mode)

    def resolve(self, rb, blk):
        if self.parked(blk) or blk.stale:
            what = 'mat lien lac' if blk.stale else 'dang do'
            self.log(f'KET: {rb.name} bi {blk.name} ({what}) chan > {WAIT_S:.0f} s', 'warn')
            # Tranh moi o xe kia dang GIU (ca o phia truoc da cap): xe mat lien
            # lac van co the dang chay not doan Nav2 da nhan.
            path = self.plan(rb, self.node_of(rb), rb.route[-1], hard=self.res[blk.name])
            if path is not None:
                self.log(f'[{rb.name}] di vong quanh {blk.name}')
                self.assign_route(rb, path, rb.final_heading, rb.mode)
                return
            if not blk.stale and self.side_step(blk, rb):
                return
        else:
            y, x = (rb, blk) if self.prio(rb) > self.prio(blk) else (blk, rb)
            why = 'cho nhau' if self.waits_for(blk, rb) else f'{blk.name} dung yen > {WAIT_S:.0f} s'
            self.log(f'KET: {rb.name} - {blk.name} {why}. {y.name} xa dich hon -> nhuong', 'warn')
            for a, b in ((y, x), (x, y)):
                if self.yield_to(a, b):
                    return
        self.log(f'KHONG GO DUOC KET giua {rb.name} va {blk.name} - dung ca hai, can nguoi xu ly', 'error')
        for r in (rb, blk):
            r.goal = None
            self.clear_route(r)

    def yield_to(self, y, x):
        """y nhuong x: lap duong vong tranh x, khong duoc thi ne sang o trong."""
        forbid = set(self.occ[x.name]) | set(self.remaining(x))
        if y.route:
            path = self.plan(y, self.node_of(y), y.route[-1], hard=forbid)
            if path is not None:
                self.log(f'[{y.name}] lap duong vong tranh {x.name}')
                self.assign_route(y, path, y.final_heading, y.mode)
                return True
        return self.side_step(y, x)

    def side_step(self, y, x):
        """y ne sang o trong gan nhat khong nam tren duong con lai cua x."""
        forbid = set(self.occ[x.name]) | set(self.remaining(x))
        if x.route:
            forbid.add(x.route[-1])
        bn, be = self.blocked_sets()
        start = self.node_of(y)
        prev = {start: None}
        q = deque([start])
        refuge = None
        while q:
            n = q.popleft()
            if n != start and n not in forbid:
                refuge = n
                break
            for nb in self.board.neighbours(*n):
                if nb in prev or nb in bn or nb in forbid and nb != start:
                    continue
                if edge_key(n, nb) in be:
                    continue
                prev[nb] = n
                q.append(nb)
        if refuge is None:
            return False
        path = [refuge]
        while prev[path[-1]] is not None:
            path.append(prev[path[-1]])
        path.reverse()
        if y.resume is None:
            if y.route:
                y.resume = (y.route[-1], y.final_heading)
            elif x.goal != start and (not x.route or x.route[-1] != start):
                y.resume = (start, y.yaw)   # xe dang do: ne xong quay ve cho cu
            # cho cu chinh la dich xe kia -> o lai cho ne, khong quay ve
        self.assign_route(y, path, None, 'yield')
        return True

    def try_resume(self, rb, now):
        if now - rb.yield_since < 1.0:
            return
        target, heading = rb.resume
        hard = set()
        for o in self.others(rb):
            if self.parked(o):
                hard |= self.occupied(o)
        path = self.plan(rb, self.node_of(rb), target, hard=hard)
        if path is None:
            return
        for o in self.others(rb):
            if not self.parked(o) and set(path) & (set(self.remaining(o)) | self.occ[o.name]):
                return          # xe kia chua qua
        rb.resume = None
        if rb.goal is None:
            rb.goal = target
        self.log(f'[{rb.name}] xe kia da qua, quay lai {self.lbl(target)}')
        self.assign_route(rb, path, heading, 'go')

    # ------------------------------------------------------------------ RViz
    def publish_markers(self):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, (n, rb) in enumerate(self.robots.items()):
            col = COLORS[i % len(COLORS)]
            m = Marker()
            m.header.frame_id, m.header.stamp = 'map', stamp
            m.ns, m.id = f'route_{n}', 0
            m.type, m.scale.x = Marker.LINE_STRIP, 0.04
            m.color = ColorRGBA(r=col[0], g=col[1], b=col[2], a=0.9)
            m.action = Marker.ADD if rb.route else Marker.DELETE
            for node in self.remaining(rb):
                x, y = self.xy(node)
                m.points.append(Point(x=float(x), y=float(y), z=0.03))
            arr.markers.append(m)
            r = Marker()
            r.header.frame_id, r.header.stamp = 'map', stamp
            r.ns, r.id = f'held_{n}', 0
            r.type = Marker.CUBE_LIST
            r.scale.x = r.scale.y = 0.14
            r.scale.z = 0.02
            r.color = ColorRGBA(r=col[0], g=col[1], b=col[2], a=0.55)
            for node in self.res.get(n, set()):
                x, y = self.xy(node)
                r.points.append(Point(x=float(x), y=float(y), z=0.01))
            r.action = Marker.ADD if r.points else Marker.DELETE
            arr.markers.append(r)
        self.marker_pub.publish(arr)


def stdin_loop(node):
    print('Nhap: "1 5 5 d" (xe 1 toi hang 5 cot 5, quay Dong), "s" dung tat ca, "q" thoat.')
    for line in sys.stdin:
        line = line.strip()
        if line in ('q', 'quit', 'exit'):
            break
        if line:
            msg = String()
            msg.data = line
            node.goal_pub.publish(msg)
    rclpy.try_shutdown()


def main():
    rclpy.init(args=sys.argv)
    argv = remove_ros_args(sys.argv)[1:]
    grid_file = argv[argv.index('--grid') + 1] if '--grid' in argv else None
    names = [a for a in argv if not a.startswith('--') and a != grid_file] or ['agv1', 'agv2']
    node = TrafficManager(names, grid_file, avoid='--no-avoid' not in argv)
    threading.Thread(target=stdin_loop, args=(node,), daemon=True).start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
