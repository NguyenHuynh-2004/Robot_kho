import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import threading
import sys
import time
import math

class GridNavigator(Node):
    def __init__(self, use_sim_time=True):
        # use_sim_time truoc day bi dong cung = True. Tren xe that khong co
        # /clock nen node se treo vinh vien ma khong bao loi gi.
        #   Mo phong: python3 grid_navigator.py
        #   Xe that : python3 grid_navigator.py --real
        super().__init__('grid_navigator', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, use_sim_time)
        ])

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.marker_pub = self.create_publisher(MarkerArray, '/grid_visualization', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.grid_size = 5
        self.spacing_x = 2.0
        self.spacing_y = 2.0
        self.offset_x = -4.0
        self.offset_y = -4.0
        self.map_received = False

        # Bán kính bao của xe: chassis 0.5m, bánh xe lồi ra tới 0.55 + 0.02 = 0.57m.
        # clearance = khoảng hở tối thiểu từ tâm xe tới vật cản để coi một vị trí là đi được.
        # (Trước đây dùng 21 pixel = 1.05m > nửa khoảng cách lưới 1.0m nên một vật cản
        #  chặn luôn cả 2 node kề nhau, khiến BFS hay báo "không có đường".)
        self.robot_radius = 0.6
        self.clearance = 0.65
        # Chỉ lấy ô vật cản THẬT (giá trị 100). Ngưỡng thấp hơn sẽ nuốt cả vùng
        # inflation của Nav2 và lại gây chặn quá tay.
        self.lethal_threshold = 99

        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos)
            
        self.blocked_nodes = [[False]*self.grid_size for _ in range(self.grid_size)]
        self.blocked_edges = set()
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.costmap_callback,
            10)
            
        self.get_logger().info("Hệ thống Điều hướng Lưới 5x5 (Auto-Grid) đã khởi động! Đang chờ bản đồ...")
        
        # Publish static grid periodically
        self.create_timer(1.0, self.publish_markers)
        
        self.waypoints = []
        self.current_path_points = []
        self.target_node = None
        self.goal_handle = None
        self.is_canceling = False
        self._replan_timer = None
        self._state_lock = threading.Lock()
        self.replan_attempts = 0
        self.max_replan_attempts = 3

    def map_callback(self, msg: OccupancyGrid):
        if self.map_received:
            return
            
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        data = msg.data
        
        min_x = float('inf')
        max_x = float('-inf')
        min_y = float('inf')
        max_y = float('-inf')
        free_found = False
        
        for y in range(height):
            for x in range(width):
                idx = x + (y * width)
                val = data[idx]
                if val == 0: # Free space
                    px = origin_x + x * resolution
                    py = origin_y + y * resolution
                    if px < min_x: min_x = px
                    if px > max_x: max_x = px
                    if py < min_y: min_y = py
                    if py > max_y: max_y = py
                    free_found = True
                    
        if not free_found:
            self.get_logger().warn("Không tìm thấy vùng free space!")
            return
            
        # Do bản đồ SLAM có thể bị nhiễu (noise) lọt ra ngoài tường,
        # chúng ta giới hạn cứng khu vực an toàn tối đa của phòng 10x10m
        # để đảm bảo lưới không bao giờ chạm tường (tường ở +-4.9m)
        min_x = max(min_x, -4.5)
        max_x = min(max_x, 4.5)
        min_y = max(min_y, -4.5)
        max_y = min(max_y, 4.5)
        
        margin = 0.5
        min_x += margin
        max_x -= margin
        min_y += margin
        max_y -= margin
        
        if max_x <= min_x or max_y <= min_y:
            self.get_logger().warn("Phòng quá nhỏ, dùng mặc định.")
            self.map_received = True
            return
            
        self.spacing_x = (max_x - min_x) / (self.grid_size - 1)
        self.spacing_y = (max_y - min_y) / (self.grid_size - 1)
        self.offset_x = min_x
        self.offset_y = min_y
        
        self.map_received = True
        self.get_logger().info(f"Đã tự tính lưới từ map! Spacing: {self.spacing_x:.2f}m x {self.spacing_y:.2f}m, Origin: ({self.offset_x:.2f}, {self.offset_y:.2f})")
        self.publish_markers()

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            qx, qy, qz, qw = t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w
            yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            return x, y, yaw
        except Exception:
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                x = t.transform.translation.x
                y = t.transform.translation.y
                qx, qy, qz, qw = t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w
                yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
                return x, y, yaw
            except Exception as e2:
                return None, None, None

    def get_closest_grid(self, x, y):
        if not self.map_received or self.spacing_x == 0:
            return 2, 2
        col = round((x - self.offset_x) / self.spacing_x)
        row = round((y - self.offset_y) / self.spacing_y)
        col = max(0, min(col, self.grid_size - 1))
        row = max(0, min(row, self.grid_size - 1))
        return row, col

    @staticmethod
    def edge_key(r1, c1, r2, c2):
        return tuple(sorted(((r1, c1), (r2, c2))))

    @staticmethod
    def point_blocked(x, y, obstacles, clearance):
        """Có vật cản nào nằm trong bán kính `clearance` quanh điểm (x, y) không?"""
        limit = clearance * clearance
        for ox, oy in obstacles:
            dx = ox - x
            dy = oy - y
            if dx * dx + dy * dy < limit:
                return True
        return False

    @staticmethod
    def segment_blocked(x1, y1, x2, y2, obstacles, clearance):
        """Có vật cản nào cách đoạn thẳng [P1, P2] dưới `clearance` không?

        Dùng khoảng cách điểm-tới-đoạn-thẳng nên không bỏ sót vật cản nằm lọt
        giữa hai điểm lấy mẫu như cách sampling theo bước 10cm trước đây.
        """
        vx = x2 - x1
        vy = y2 - y1
        length_sq = vx * vx + vy * vy
        limit = clearance * clearance
        for ox, oy in obstacles:
            if length_sq == 0.0:
                t = 0.0
            else:
                t = ((ox - x1) * vx + (oy - y1) * vy) / length_sq
                t = max(0.0, min(1.0, t))
            dx = ox - (x1 + t * vx)
            dy = oy - (y1 + t * vy)
            if dx * dx + dy * dy < limit:
                return True
        return False

    def collect_obstacles(self, msg: OccupancyGrid):
        """Toạ độ thực (x, y) của các ô costmap là vật cản thật, đã lọc theo vùng lưới.

        Ô nằm xa lưới hơn `clearance` thì không thể chặn node hay cạnh nào, nên bỏ
        qua luôn cho nhẹ (thay cho ngưỡng cứng ±4.1m viết tay trước đây).
        """
        info = msg.info
        width = info.width
        resolution = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y

        pad = self.clearance + resolution
        grid_min_x = self.offset_x - pad
        grid_max_x = self.offset_x + (self.grid_size - 1) * self.spacing_x + pad
        grid_min_y = self.offset_y - pad
        grid_max_y = self.offset_y + (self.grid_size - 1) * self.spacing_y + pad

        obstacles = []
        for idx, val in enumerate(msg.data):
            if val <= self.lethal_threshold:
                continue
            cx = idx % width
            cy = idx // width
            px = origin_x + (cx + 0.5) * resolution
            py = origin_y + (cy + 0.5) * resolution
            if grid_min_x <= px <= grid_max_x and grid_min_y <= py <= grid_max_y:
                obstacles.append((px, py))
        return obstacles

    def costmap_callback(self, msg: OccupancyGrid):
        if not self.map_received:
            return

        obstacles = self.collect_obstacles(msg)

        changed = False
        for r in range(self.grid_size):
            for c in range(self.grid_size):
                x, y = self.get_coordinates(r, c)
                is_blocked = self.point_blocked(x, y, obstacles, self.clearance)
                if self.blocked_nodes[r][c] != is_blocked:
                    self.blocked_nodes[r][c] = is_blocked
                    changed = True

        # Cạnh nối 2 node kề nhau cũng phải đủ rộng cho xe chui qua: vật cản nằm
        # đúng giữa 2 node thì không node nào bị chặn nhưng lối đi thì tắc.
        new_blocked_edges = set()
        for r in range(self.grid_size):
            for c in range(self.grid_size):
                x1, y1 = self.get_coordinates(r, c)
                for dr, dc in ((0, 1), (1, 0)):
                    nr, nc = r + dr, c + dc
                    if nr >= self.grid_size or nc >= self.grid_size:
                        continue
                    x2, y2 = self.get_coordinates(nr, nc)
                    if self.segment_blocked(x1, y1, x2, y2, obstacles, self.clearance):
                        new_blocked_edges.add(self.edge_key(r, c, nr, nc))

        if new_blocked_edges != self.blocked_edges:
            self.blocked_edges = new_blocked_edges
            changed = True

        if changed:
            self.publish_markers()

        # Kiểm tra xem lộ trình hiện tại có bị chặn không
        if self.target_node is not None and not self.is_canceling:
            if self.is_path_blocked(obstacles):
                self.get_logger().warn("Phát hiện vật cản trên đường đi! Đang tiến hành dừng xe khẩn cấp...")
                self.cancel_current_goal_and_replan()

    def is_path_blocked(self, obstacles):
        if not self.current_path_points:
            return False

        rx, ry, _ = self.get_robot_pose()
        if rx is None:
            return False

        # Dùng robot_radius (không phải clearance) để chỉ huỷ lộ trình khi lối đi
        # thực sự hẹp hơn thân xe, tránh replan liên tục vì nhiễu lidar.
        pts = [(rx, ry)] + self.current_path_points
        for i in range(len(pts) - 1):
            x1, y1 = pts[i]
            x2, y2 = pts[i + 1]
            if self.segment_blocked(x1, y1, x2, y2, obstacles, self.robot_radius):
                return True
        return False

    def get_coordinates(self, row, col):
        x = self.offset_x + col * self.spacing_x
        y = self.offset_y + row * self.spacing_y
        return float(x), float(y)

    @staticmethod
    def normalize_angle(angle):
        """Đưa góc về [-pi, pi] để so sánh hướng không nhảy sai ở biên ±180°."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def publish_markers(self):
        marker_array = MarkerArray()
        
        # 1. Vẽ các điểm (Nodes)
        nodes_marker = Marker()
        nodes_marker.header.frame_id = 'map'
        nodes_marker.header.stamp = self.get_clock().now().to_msg()
        nodes_marker.ns = 'grid_nodes'
        nodes_marker.id = 0
        nodes_marker.type = Marker.SPHERE_LIST
        nodes_marker.action = Marker.ADD
        nodes_marker.scale.x = 0.15
        nodes_marker.scale.y = 0.15
        nodes_marker.scale.z = 0.15
        nodes_marker.color.r = 0.0
        nodes_marker.color.g = 0.5
        nodes_marker.color.b = 1.0
        nodes_marker.color.a = 1.0

        # 2. Vẽ các đường nối (Edges)
        edges_marker = Marker()
        edges_marker.header.frame_id = 'map'
        edges_marker.header.stamp = self.get_clock().now().to_msg()
        edges_marker.ns = 'grid_edges'
        edges_marker.id = 1
        edges_marker.type = Marker.LINE_LIST
        edges_marker.action = Marker.ADD
        edges_marker.scale.x = 0.02
        edges_marker.color.r = 0.0
        edges_marker.color.g = 0.8
        edges_marker.color.b = 1.0
        edges_marker.color.a = 0.5

        for row in range(self.grid_size):
            for col in range(self.grid_size):
                if self.blocked_nodes[row][col]:
                    continue
                    
                x, y = self.get_coordinates(row, col)
                p = Point()
                p.x = x
                p.y = y
                p.z = 0.0
                nodes_marker.points.append(p)
                
                # Nối với điểm bên phải
                if (col < self.grid_size - 1
                        and not self.blocked_nodes[row][col + 1]
                        and self.edge_key(row, col, row, col + 1) not in self.blocked_edges):
                    p_right = Point()
                    p_right.x, p_right.y = self.get_coordinates(row, col + 1)
                    p_right.z = 0.0
                    edges_marker.points.append(p)
                    edges_marker.points.append(p_right)

                # Nối với điểm phía trên
                if (row < self.grid_size - 1
                        and not self.blocked_nodes[row + 1][col]
                        and self.edge_key(row, col, row + 1, col) not in self.blocked_edges):
                    p_up = Point()
                    p_up.x, p_up.y = self.get_coordinates(row + 1, col)
                    p_up.z = 0.0
                    edges_marker.points.append(p)
                    edges_marker.points.append(p_up)

        marker_array.markers.append(nodes_marker)
        marker_array.markers.append(edges_marker)

        # 3. Vẽ Target Node (nếu có)
        if self.target_node is not None:
            target_marker = Marker()
            target_marker.header.frame_id = 'map'
            target_marker.header.stamp = self.get_clock().now().to_msg()
            target_marker.ns = 'target_node'
            target_marker.id = 2
            target_marker.type = Marker.SPHERE
            target_marker.action = Marker.ADD
            target_marker.pose.position.x = self.target_node[0]
            target_marker.pose.position.y = self.target_node[1]
            target_marker.pose.position.z = 0.05
            target_marker.scale.x = 0.3
            target_marker.scale.y = 0.3
            target_marker.scale.z = 0.3
            target_marker.color.r = 1.0
            target_marker.color.g = 0.5
            target_marker.color.b = 0.0
            target_marker.color.a = 1.0
            marker_array.markers.append(target_marker)

        # 4. Vẽ Path (nếu có)
        if len(self.current_path_points) > 1:
            path_marker = Marker()
            path_marker.header.frame_id = 'map'
            path_marker.header.stamp = self.get_clock().now().to_msg()
            path_marker.ns = 'planned_path'
            path_marker.id = 3
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.scale.x = 0.08
            path_marker.color.r = 0.0
            path_marker.color.g = 1.0
            path_marker.color.b = 0.0
            path_marker.color.a = 0.8
            
            for pt in self.current_path_points:
                p = Point()
                p.x = pt[0]
                p.y = pt[1]
                p.z = 0.02
                path_marker.points.append(p)
                
            marker_array.markers.append(path_marker)
        else:
            # Xóa path cũ
            path_marker = Marker()
            path_marker.header.frame_id = 'map'
            path_marker.ns = 'planned_path'
            path_marker.id = 3
            path_marker.action = Marker.DELETE
            marker_array.markers.append(path_marker)

        self.marker_pub.publish(marker_array)


    def find_path_bfs(self, start_r, start_c, target_r, target_c):
        if self.blocked_nodes[start_r][start_c] or self.blocked_nodes[target_r][target_c]:
            return None
            
        queue = [(start_r, start_c)]
        visited = set()
        visited.add((start_r, start_c))
        parent = {}
        
        while queue:
            curr_r, curr_c = queue.pop(0)
            if curr_r == target_r and curr_c == target_c:
                break
                
            for dr, dc in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                nr, nc = curr_r + dr, curr_c + dc
                if 0 <= nr < self.grid_size and 0 <= nc < self.grid_size:
                    if self.blocked_nodes[nr][nc] or (nr, nc) in visited:
                        continue
                    if self.edge_key(curr_r, curr_c, nr, nc) in self.blocked_edges:
                        continue
                    visited.add((nr, nc))
                    parent[(nr, nc)] = (curr_r, curr_c)
                    queue.append((nr, nc))
                        
        if (target_r, target_c) not in parent and (start_r, start_c) != (target_r, target_c):
            return None
            
        path = []
        curr = (target_r, target_c)
        while curr != (start_r, start_c):
            path.append(curr)
            curr = parent[curr]
        path.reverse()
        return path

    def send_grid_goal(self, start_row, start_col, target_row, target_col, start_yaw=None):
        self._action_client.wait_for_server()

        self.waypoints = []
        self.current_path_points = []

        start_x, start_y = self.get_coordinates(start_row, start_col)

        path_coords = self.find_path_bfs(start_row, start_col, target_row, target_col)
        if path_coords is None:
            self.abort_route('Đích đến bị vật cản chặn và không có đường vòng')
            return
            
        prev_x, prev_y = start_x, start_y
        prev_yaw = start_yaw
        
        for (nr, nc) in path_coords:
            nx, ny = self.get_coordinates(nr, nc)
            yaw = math.atan2(ny - prev_y, nx - prev_x)
            
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            qw, qx, qy, qz = cy, 0.0, 0.0, sy
            
            if prev_yaw is not None and abs(self.normalize_angle(yaw - prev_yaw)) > 0.01:
                # Xe đang chuyển hướng -> Chèn 1 waypoint xoay tại chỗ (Spin in place) ở góc cua
                self.waypoints.append((prev_x, prev_y, qw, qx, qy, qz))
            
            self.waypoints.append((nx, ny, qw, qx, qy, qz))

            prev_x, prev_y = nx, ny
            prev_yaw = yaw
             
        target_x, target_y = self.get_coordinates(target_row, target_col)
        self.target_node = (target_x, target_y)

        self.refresh_path_points()
        self.publish_markers()
        self.get_logger().info(f'Lộ trình ({len(self.waypoints)} bước) đã tạo!')
        self.send_next_waypoint()

    def refresh_path_points(self):
        """Dựng lại đường còn phải đi từ danh sách waypoint chưa tới.

        Trước đây current_path_points bị pop riêng mỗi khi tới một waypoint, nhưng
        waypoints còn chứa cả các điểm xoay tại chỗ ở khúc cua nên hai danh sách
        lệch nhau: đường dùng để kiểm tra va chạm cạn dần rồi chỉ còn 1 đoạn cũ
        trước khi xe tới nơi, khiến việc phát hiện vật cản tê liệt ở cuối lộ trình.
        """
        pts = []
        for wp in self.waypoints:
            x, y = wp[0], wp[1]
            if not pts or math.hypot(pts[-1][0] - x, pts[-1][1] - y) > 1e-6:
                pts.append((x, y))
        self.current_path_points = pts

    def cancel_current_goal_and_replan(self):
        # MultiThreadedExecutor cho phep costmap_callback va timer replan chay
        # dong thoi. Khong co khoa nay thi mot vat can se kich hoat nhieu lan
        # huy/replan chong cheo: log hien dau thoi gian dao nguoc va xe bi tinh
        # lai duong 4-5 lan lien tiep truoc khi on dinh.
        with self._state_lock:
            if self.is_canceling:
                return          # da co mot luot huy dang chay roi
            self.is_canceling = True
            handle = self.goal_handle

        if handle is not None:
            cancel_future = handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.schedule_replan()

    def cancel_done_callback(self, future):
        self.get_logger().info('Đã dừng xe thành công. Đang tính toán đường vòng...')
        self.schedule_replan()

    def schedule_replan(self, delay=1.0):
        """Hẹn giờ replan sau `delay` giây cho xe dừng hẳn.

        Dùng timer thay vì time.sleep() vì hàm này chạy trong callback của executor;
        sleep ở đây sẽ khoá luôn một thread và làm nghẽn costmap_callback.
        """
        if self._replan_timer is not None:
            self.destroy_timer(self._replan_timer)
            self._replan_timer = None
        self._replan_timer = self.create_timer(delay, self.replan_timer_callback)

    def replan_timer_callback(self):
        if self._replan_timer is not None:
            self.destroy_timer(self._replan_timer)
            self._replan_timer = None
        self.replan_after_cancel()

    def replan_after_cancel(self):
        if self.target_node is None:
            # Lộ trình đã bị huỷ hoặc đã tới nơi trong lúc chờ -> không còn gì để replan.
            self.is_canceling = False
            return

        rx, ry, ryaw = self.get_robot_pose()
        if rx is None:
            self.get_logger().warn("Chưa lấy được vị trí xe từ TF, thử replan lại sau 1 giây...")
            self.schedule_replan()
            return

        current_row, current_col = self.get_closest_grid(rx, ry)
        target_row, target_col = self.get_closest_grid(self.target_node[0], self.target_node[1])
        self.get_logger().info(f"Bắt đầu Replan từ ({current_row}, {current_col}) đến ({target_row}, {target_col})")
        # GIU is_canceling = True cho toi khi Nav2 chap nhan goal moi.
        # Ha co o day se mo cua cho costmap_callback ke tiep kich hoat mot luot
        # huy nua tren goal_handle da chet.
        self.send_grid_goal(current_row, current_col, target_row, target_col, ryaw)


    def send_next_waypoint(self):
        if not self.waypoints:
            self.get_logger().info('Đã đến nơi thành công toàn bộ lộ trình!')
            self.current_path_points = []
            self.target_node = None
            self.is_canceling = False
            self.publish_markers()
            print("\n=== ĐIỀU KHIỂN AGV SA BÀN 5x5 ===")
            print("Xe đang chờ lệnh tiếp theo. Hãy nhập trên terminal...")
            return
            
        nx, ny, qw, qx, qy, qz = self.waypoints.pop(0)
        self.refresh_path_points()
        self.get_logger().info(f'Đang di chuyển tới tọa độ: ({nx:.2f}, {ny:.2f})')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = rclpy.time.Time().to_msg()
        goal_msg.pose.pose.position.x = float(nx)
        goal_msg.pose.pose.position.y = float(ny)
        goal_msg.pose.pose.orientation.w = float(qw)
        goal_msg.pose.pose.orientation.x = float(qx)
        goal_msg.pose.pose.orientation.y = float(qy)
        goal_msg.pose.pose.orientation.z = float(qz)

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            # Trước đây chỉ log rồi return: waypoints và target_node vẫn còn nên
            # vòng chờ trong input_thread treo vĩnh viễn, phải Ctrl-C mới thoát.
            self.abort_route('Nav2 từ chối điểm đến')
            return

        self.goal_handle = goal_handle
        # Goal moi da song -> ket thuc luot huy, cho phep giam sat vat can tro lai.
        self.is_canceling = False
        self.get_logger().info('Điểm đến được chấp nhận, xe đang di chuyển...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        if self.is_canceling:
            return

        # Trước đây kết quả bị bỏ qua hoàn toàn: Nav2 kẹt hay hết giờ vẫn bị coi là
        # "đã tới" và xe nhảy sang waypoint kế tiếp từ một vị trí sai.
        status = getattr(future.result(), 'status', None)
        if status == GoalStatus.STATUS_CANCELED:
            return  # luồng huỷ đã lo phần replan
        if status is not None and status != GoalStatus.STATUS_SUCCEEDED:
            self.replan_attempts += 1
            if self.replan_attempts > self.max_replan_attempts:
                self.abort_route(f"Nav2 thất bại {self.replan_attempts} lần liên tiếp")
                return
            self.get_logger().warn(
                f'Nav2 không tới được điểm trung gian (status={status}). '
                f'Thử tính lại đường lần {self.replan_attempts}/{self.max_replan_attempts}...')
            self.is_canceling = True
            self.goal_handle = None
            self.schedule_replan()
            return

        self.replan_attempts = 0
        self.get_logger().info('Đã tới điểm trung gian.')
        self.send_next_waypoint()

    def abort_route(self, reason):
        """Dừng hẳn lộ trình và trả quyền điều khiển về cho người dùng.

        Cần thiết vì input_thread đang chờ trên `while waypoints or target_node`;
        nếu không dọn trạng thái ở đây thì terminal treo vĩnh viễn.
        """
        self.get_logger().error(f'Huỷ lộ trình: {reason}')
        self.waypoints = []
        self.current_path_points = []
        self.target_node = None
        self.goal_handle = None
        self.is_canceling = False
        self.replan_attempts = 0
        self.publish_markers()
        print(f"\n[LỖI] {reason}. Xe đã dừng — hãy nhập lệnh mới.")

def input_thread(navigator):
    print("Đang chờ khởi tạo hệ thống và nhận bản đồ...")
    
    timeout = 10  # 10 lần x 0.5s = 5 giây
    while rclpy.ok() and not navigator.map_received and timeout > 0:
        time.sleep(0.5)
        timeout -= 1
        
    if not navigator.map_received:
        print("\n[CẢNH BÁO] Không nhận được bản đồ từ Navigation2 sau 5 giây!")
        print("=> Tự động dùng kích thước sa bàn chuẩn (10x10m).")
        navigator.map_received = True
    else:
        # Đợi thêm 1 chút để TF buffer nhận đủ dữ liệu
        time.sleep(1.0)
    
    while rclpy.ok():
        print("\n=== ĐIỀU KHIỂN AGV SA BÀN 5x5 ===")
        
        rx, ry, ryaw = navigator.get_robot_pose()
        if rx is not None:
            current_row, current_col = navigator.get_closest_grid(rx, ry)
            start_yaw = ryaw
        else:
            current_row, current_col = 2, 2
            start_yaw = None
            
        try:
            target_row_str = input(f"Vị trí xe hiện tại (Row {current_row}, Col {current_col}). Nhập Hàng ĐÍCH (0-4) hoặc 'q' để thoát: ")
            if target_row_str.lower() == 'q':
                rclpy.shutdown()
                break
            target_row = int(target_row_str)
            target_col = int(input("Nhập Cột ĐÍCH (0-4): "))
            
            if not (0 <= target_row <= 4 and 0 <= target_col <= 4):
                print("Lỗi: Tọa độ ngoài sa bàn. Chỉ chấp nhận từ 0 đến 4.")
                continue
                
            navigator.send_grid_goal(current_row, current_col, target_row, target_col, start_yaw)
            
            # Tạm thời khóa thread input để xe chạy
            while len(navigator.waypoints) > 0 or navigator.target_node is not None:
                 time.sleep(1)
                 if not rclpy.ok():
                     break
                     
        except ValueError:
            print("Lỗi: Vui lòng chỉ nhập số nguyên từ 0 đến 4!")
        except EOFError:
            break

def main(args=None):
    rclpy.init(args=args)

    # --real: chay tren xe that (dung dong ho he thong thay vi /clock cua Gazebo)
    use_sim_time = '--real' not in sys.argv
    print(f"Che do: {'MO PHONG (use_sim_time=true)' if use_sim_time else 'XE THAT (use_sim_time=false)'}")

    navigator = GridNavigator(use_sim_time=use_sim_time)

    # Chạy vòng lặp spin của ROS trong background thread
    executor = MultiThreadedExecutor()
    executor.add_node(navigator)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    # Thread chính dùng để nhận input liên tục
    input_thread(navigator)
    
    navigator.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    ros_thread.join()

if __name__ == '__main__':
    main()
