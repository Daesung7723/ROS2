import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, SetPen
from std_srvs.srv import Empty
import math
import random
from enum import Enum
from dataclasses import dataclass
from collections import deque

@dataclass
class Rect:
    """사각형 영역을 정의하는 데이터 클래스"""
    x_min: float
    x_max: float
    y_min: float
    y_max: float

def normalize_angle(angle):
    """각도를 -pi ~ +pi 범위로 정규화합니다."""
    return math.atan2(math.sin(angle), math.cos(angle))

class CellState(Enum):
    """지도의 각 셀 상태를 정의하는 열거형"""
    UNEXPLORED = 0
    EXPLORED = 1
    WALL = 2

class TurtleState(Enum):
    """터틀의 현재 동작 상태를 정의하는 열거형 클래스"""
    RUNNING = 0
    RESETTING = 1
    AVOIDING_START = 2
    AVOIDING_BACKWARD = 3
    AVOIDING_ROTATE = 4

class Map:
    """미로 지도를 관리하는 클래스"""
    def __init__(self, size, resolution):
        self.size = size
        self.resolution = resolution
        self.grid = [[CellState.UNEXPLORED for _ in range(size)] for _ in range(size)]

    def world_to_grid(self, x, y):
        """월드 좌표를 그리드 좌표로 변환합니다."""
        grid_x = int(x / self.resolution)
        grid_y = int(y / self.resolution)
        return grid_x, grid_y

    def is_valid(self, grid_x, grid_y):
        """그리드 좌표가 유효한 범위 내에 있는지 확인합니다."""
        return 0 <= grid_x < self.size and 0 <= grid_y < self.size

    def set_state(self, x, y, state):
        """주어진 월드 좌표에 해당하는 셀의 상태를 설정합니다."""
        grid_x, grid_y = self.world_to_grid(x, y)
        if self.is_valid(grid_x, grid_y):
            self.grid[grid_y][grid_x] = state

    def find_nearest_unexplored(self, start_x, start_y):
        """BFS를 사용하여 시작 지점에서 가장 가까운 미탐색 영역을 찾습니다."""
        start_grid_x, start_grid_y = self.world_to_grid(start_x, start_y)
        if not self.is_valid(start_grid_x, start_grid_y):
            return None

        queue = deque([(start_grid_x, start_grid_y)])
        visited = set([(start_grid_x, start_grid_y)])

        while queue:
            gx, gy = queue.popleft()

            if self.grid[gy][gx] == CellState.UNEXPLORED:
                # 그리드 중심의 월드 좌표 반환
                return (gx + 0.5) * self.resolution, (gy + 0.5) * self.resolution

            # 4방향 탐색 (상, 하, 좌, 우)
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nx, ny = gx + dx, gy + dy

                if self.is_valid(nx, ny) and (nx, ny) not in visited and self.grid[ny][nx] != CellState.WALL:
                    visited.add((nx, ny))
                    queue.append((nx, ny))
        return None # 탐색할 곳 없음

class MazeEscapeNode(Node):
    """
    Turtlesim을 사용하여 미로를 탈출하는 ROS2 노드.
    - 지도를 기억하며 탐색하지 않은 영역으로 우선 이동
    - 경계 및 벽에 닿으면 회피 기동 후, 해당 위치를 벽으로 기록
    - 목적지에 닿으면 다시 시작
    """
    def __init__(self):
        super().__init__('maze_escape_map_node')

        # 상수 정의
        self.START_POSE = Pose(x=1.5, y=1.5)
        self.GOAL_AREA = Rect(x_min=8.0, x_max=9.0, y_min=8.0, y_max=9.0)
        self.BOUNDARIES = Rect(x_min=1.0, x_max=9.0, y_min=1.0, y_max=9.0)
        self.WALLS = [
            Rect(x_min=3.0, x_max=4.0, y_min=1.0, y_max=6.0),
            Rect(x_min=6.0, x_max=7.0, y_min=3.0, y_max=9.0)
        ]
        self.BACKWARD_SPEED = 2.0
        self.ANGULAR_SPEED = 2.0
        self.LINEAR_SPEED = 2.0
        self.SAFETY_MARGIN = 0.2
        
        # 펜 설정
        self.PEN_R, self.PEN_G, self.PEN_B = 0, 250, 0
        self.PEN_WIDTH = 10

        # 지도 초기화
        self.map = Map(size=12, resolution=1.0)

        # 현재 터틀의 위치와 상태
        self.current_pose = None
        self.state = TurtleState.RESETTING
        self.target_pos = None # 탐색 목표 지점

        # ROS2 인터페이스 생성
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        # 서비스 클라이언트 생성
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.clear_client = self.create_client(Empty, '/clear')
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        # 서비스 서버가 준비될 때까지 대기
        for client in [self.teleport_client, self.clear_client, self.set_pen_client]:
            client.wait_for_service()

        # 메인 로직을 실행할 타이머
        self.timer = self.create_timer(0.02, self.main_loop)

        self.get_logger().info('미로 탈출(지도) 노드가 시작되었습니다. 초기화 중...')
        self.reset_simulation()

    def pose_callback(self, msg):
        """/turtle1/pose 토픽 콜백 함수. 현재 위치를 업데이트하고 지도를 갱신합니다."""
        self.current_pose = msg
        # 현재 위치를 탐색 완료로 표시
        if self.state == TurtleState.RUNNING:
            self.map.set_state(msg.x, msg.y, CellState.EXPLORED)

    def main_loop(self):
        """메인 제어 루프. 타이머에 의해 주기적으로 호출됩니다."""
        if self.current_pose is None or self.state == TurtleState.RESETTING:
            return

        if self.state == TurtleState.RUNNING:
            if self.is_in_goal_area():
                self.get_logger().info('목표 지점 도달! 시뮬레이션을 리셋합니다.')
                self.reset_simulation()
                return

            if self.is_colliding():
                self.get_logger().info('경계 또는 벽에 충돌! 회피 기동을 시작합니다.')
                # 충돌 지점을 벽으로 기록
                coll_x = self.current_pose.x + self.SAFETY_MARGIN * math.cos(self.current_pose.theta)
                coll_y = self.current_pose.y + self.SAFETY_MARGIN * math.sin(self.current_pose.theta)
                self.map.set_state(coll_x, coll_y, CellState.WALL)
                
                self.stop_turtle()
                self.state = TurtleState.AVOIDING_START
            else:
                self.explore_unvisited_areas()

        elif self.state == TurtleState.AVOIDING_START:
            self.avoid_collision_start()

        elif self.state == TurtleState.AVOIDING_BACKWARD:
            if not self.is_colliding():
                self.get_logger().info('안전 영역 복귀. 회전을 시작합니다.')
                self.start_rotation_callback()
            else:
                twist_msg = Twist()
                twist_msg.linear.x = -self.BACKWARD_SPEED
                self.cmd_vel_publisher.publish(twist_msg)

        elif self.state == TurtleState.AVOIDING_ROTATE:
            self.avoid_collision_rotate()

    def reset_simulation(self):
        """시뮬레이션을 초기 상태로 리셋합니다."""
        self.state = TurtleState.RESETTING
        self.stop_turtle()
        self.map = Map(size=12, resolution=1.0) # 지도 초기화
        self._start_teleport_sequence()

    def _start_teleport_sequence(self):
        """리셋 시퀀스: 펜 들기 -> 텔레포트 -> 화면 클리어 -> 펜 내리기"""
        pen_req = SetPen.Request(off=1)
        future = self.set_pen_client.call_async(pen_req)
        future.add_done_callback(self._pen_up_done_callback)

    def _pen_up_done_callback(self, future):
        try: future.result()
        except Exception as e: self.get_logger().error(f'펜 들기 실패: {e}')
        
        teleport_req = TeleportAbsolute.Request(
            x=self.START_POSE.x,
            y=self.START_POSE.y,
            theta=random.uniform(0, 2 * math.pi)
        )
        teleport_future = self.teleport_client.call_async(teleport_req)
        teleport_future.add_done_callback(self._teleport_done_callback)

    def _teleport_done_callback(self, future):
        try: future.result()
        except Exception as e: self.get_logger().error(f'텔레포트 실패: {e}')

        clear_future = self.clear_client.call_async(Empty.Request())
        clear_future.add_done_callback(self._clear_done_callback)

    def _clear_done_callback(self, future):
        try: future.result()
        except Exception as e: self.get_logger().error(f'화면 지우기 실패: {e}')
        
        self._set_turtle1_pen_and_start_running()

    def _set_turtle1_pen_and_start_running(self):
        """turtle1의 펜을 설정하고 주행 상태로 전환합니다."""
        pen_req = SetPen.Request(
            off=0, r=self.PEN_R, g=self.PEN_G, b=self.PEN_B, width=self.PEN_WIDTH
        )
        self.set_pen_client.call_async(pen_req)

        self.get_logger().info('초기화 완료. 지도 기반 탐색을 시작합니다.')
        self.state = TurtleState.RUNNING

    def is_in_goal_area(self):
        """터틀이 목표 영역 내에 있는지 확인합니다."""
        return (self.GOAL_AREA.x_min <= self.current_pose.x <= self.GOAL_AREA.x_max and
                self.GOAL_AREA.y_min <= self.current_pose.y <= self.GOAL_AREA.y_max)

    def is_colliding(self):
        """터틀이 경계 또는 벽에 충돌했는지 확인합니다."""
        margin = self.SAFETY_MARGIN
        pose = self.current_pose

        if not (self.BOUNDARIES.x_min + margin < pose.x < self.BOUNDARIES.x_max - margin and
                self.BOUNDARIES.y_min + margin < pose.y < self.BOUNDARIES.y_max - margin):
            return True
        
        for wall in self.WALLS:
            if (wall.x_min - margin < pose.x < wall.x_max + margin and
                wall.y_min - margin < pose.y < wall.y_max + margin):
                return True
        return False

    def avoid_collision_start(self):
        """충돌 회피 동작을 시작합니다. (정지 및 후진)"""
        self.stop_turtle()
        self.state = TurtleState.AVOIDING_BACKWARD
        twist_msg = Twist()
        twist_msg.linear.x = -self.BACKWARD_SPEED
        self.cmd_vel_publisher.publish(twist_msg)

    def start_rotation_callback(self):
        """후진 완료 후 호출되는 콜백. 회전을 시작합니다."""
        self.stop_turtle()
        self.state = TurtleState.AVOIDING_ROTATE
        random_rotation = random.uniform(math.pi / 2, 3 * math.pi / 2)
        self.target_theta = normalize_angle(self.current_pose.theta + random_rotation)

    def avoid_collision_rotate(self):
        """목표 각도에 도달할 때까지 회전합니다."""
        angle_diff = normalize_angle(self.target_theta - self.current_pose.theta)

        if abs(angle_diff) > 0.1:
            twist_msg = Twist()
            twist_msg.angular.z = self.ANGULAR_SPEED if angle_diff > 0 else -self.ANGULAR_SPEED
            self.cmd_vel_publisher.publish(twist_msg)
        else:
            self.stop_turtle()
            self.get_logger().info('회피 기동 완료. 주행을 재개합니다.')
            self.state = TurtleState.RUNNING
            self.target_pos = None # 회피 후 목표 재설정

    def explore_unvisited_areas(self):
        """가장 가까운 미탐색 영역으로 이동합니다."""
        # 목표가 없거나, 목표에 도달했다면 새로운 목표를 찾음
        if self.target_pos is None or \
           math.dist((self.current_pose.x, self.current_pose.y), self.target_pos) < 0.5:
            self.target_pos = self.map.find_nearest_unexplored(self.current_pose.x, self.current_pose.y)

        if self.target_pos is None:
            # 탐색할 곳이 없으면 랜덤 주행
            self.get_logger().info('탐색할 영역이 없습니다. 랜덤 주행으로 전환합니다.', throttle_duration_sec=5)
            self.move_randomly()
            return

        # 목표 지점을 향해 이동
        target_x, target_y = self.target_pos
        angle_to_target = math.atan2(target_y - self.current_pose.y, target_x - self.current_pose.x)
        angle_diff = normalize_angle(angle_to_target - self.current_pose.theta)

        twist_msg = Twist()
        # 목표 방향과 비슷해지면 직진
        if abs(angle_diff) < 0.2:
            twist_msg.linear.x = self.LINEAR_SPEED
            twist_msg.angular.z = 0.0
        else: # 방향이 다르면 회전
            twist_msg.linear.x = self.LINEAR_SPEED * 0.5 # 회전 시 속도 감소
            twist_msg.angular.z = self.ANGULAR_SPEED if angle_diff > 0 else -self.ANGULAR_SPEED
        
        self.cmd_vel_publisher.publish(twist_msg)

    def move_randomly(self):
        """랜덤 주행 (Fallback용)"""
        twist_msg = Twist()
        twist_msg.linear.x = random.uniform(1.5, 2.5)
        twist_msg.angular.z = random.uniform(-2.5, 2.5)
        self.cmd_vel_publisher.publish(twist_msg)

    def stop_turtle(self):
        """터틀을 정지시킵니다."""
        twist_msg = Twist()
        self.cmd_vel_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = MazeEscapeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('사용자에 의해 노드가 종료됩니다.')
    finally:
        if node:
            node.stop_turtle()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
