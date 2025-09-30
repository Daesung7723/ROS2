import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, SetPen, Spawn, Kill
from std_srvs.srv import Empty
import math
import random
from enum import Enum
from dataclasses import dataclass
import asyncio
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

class TurtleState(Enum):
    """터틀의 현재 동작 상태를 정의하는 열거형 클래스"""
    RUNNING = 0         # 일반 주행
    RESETTING = 1       # 리셋 중 (텔레포트, 클리어 등)
    AVOIDING_START = 2  # 회피 시작
    AVOIDING_BACKWARD = 3 # 후진 중
    AVOIDING_ROTATE = 4   # 회전 중

class MazeEscapeNode(Node):
    """
    Turtlesim을 사용하여 미로를 탈출하는 ROS2 노드.
    - 시작 지점(1.5, 1.5)에서 랜덤 주행 시작
    - 경계 및 벽에 닿으면 후진 후 180도 회전
    - 목적지(8~9, 8~9)에 닿으면 다시 시작 지점으로 복귀
    """
    def __init__(self):
        super().__init__('maze_escape_node')

        # 상수 정의
        self.START_POSE = Pose(x=1.5, y=1.5)
        self.GOAL_AREA = Rect(x_min=8.0, x_max=9.0, y_min=8.0, y_max=9.0)
        self.BOUNDARIES = Rect(x_min=1.0, x_max=9.0, y_min=1.0, y_max=9.0)
        self.WALLS = [
            Rect(x_min=3.0, x_max=4.0, y_min=1.0, y_max=6.0),
            Rect(x_min=6.0, x_max=7.0, y_min=3.0, y_max=9.0)
        ]
        self.BACKWARD_SPEED = 2.0
        self.BACKWARD_DISTANCE = 0.5
        self.ANGULAR_SPEED = 2.0 # 회전 속도
        self.SAFETY_MARGIN = 0.2 # 안전 마진
        
        # 펜 설정
        self.PEN_R, self.PEN_G, self.PEN_B = 0, 250, 0
        self.PEN_WIDTH = 10

        # 현재 터틀의 위치와 상태
        self.current_pose = None
        self.state = TurtleState.RESETTING
        self.one_shot_timer = None # 일회성 타이머를 저장할 변수

        # ROS2 인터페이스 생성
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        # 서비스 클라이언트 생성
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.clear_client = self.create_client(Empty, '/clear')
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        # 서비스 서버가 준비될 때까지 대기
        self.teleport_client.wait_for_service()
        self.clear_client.wait_for_service()
        self.set_pen_client.wait_for_service()

        # 메인 로직을 실행할 타이머
        self.timer = self.create_timer(0.02, self.main_loop)

        self.get_logger().info('미로 탈출 노드가 시작되었습니다. 초기화 중...')
        self.reset_simulation()

    def pose_callback(self, msg):
        """/turtle1/pose 토픽 콜백 함수. 현재 위치를 업데이트합니다."""
        self.current_pose = msg

    def main_loop(self):
        """메인 제어 루프. 타이머에 의해 주기적으로 호출됩니다."""
        if self.current_pose is None or self.state == TurtleState.RESETTING:
            return

        if self.state == TurtleState.RUNNING:
            # 1. 목표 지점 도달 확인
            if self.is_in_goal_area():
                self.get_logger().info('목표 지점 도달! 시뮬레이션을 리셋합니다.')
                self.reset_simulation()
                return
            # 2. 경계 또는 벽 충돌 확인
            if self.is_colliding():
                self.get_logger().info('경계 또는 벽에 충돌! 회피 기동을 시작합니다.')
                self.stop_turtle() # 충돌 감지 즉시 정지
                self.state = TurtleState.AVOIDING_START
            else:
                # 3. 랜덤 주행
                self.move_randomly()

        elif self.state == TurtleState.AVOIDING_START:
            self.avoid_collision_start()

        elif self.state == TurtleState.AVOIDING_BACKWARD:
            # 안전 영역으로 돌아왔는지 확인
            if not self.is_colliding():
                self.get_logger().info('안전 영역 복귀. 회전을 시작합니다.')
                self.start_rotation_callback()
            else:
                # 아직 안전하지 않으면 계속 후진
                twist_msg = Twist()
                twist_msg.linear.x = -self.BACKWARD_SPEED
                self.cmd_vel_publisher.publish(twist_msg)

        elif self.state == TurtleState.AVOIDING_ROTATE:
            self.avoid_collision_rotate()

    def reset_simulation(self):
        """시뮬레이션을 초기 상태로 리셋합니다."""
        self.state = TurtleState.RESETTING
        self.stop_turtle()

        # 터틀을 시작 지점으로 텔레포트하는 시퀀스 시작
        self._start_teleport_sequence()

    def _start_teleport_sequence(self):
        """turtle1을 시작 지점으로 텔레포트하는 시퀀스를 시작합니다."""
        # 1. 텔레포트 시 경로가 그려지지 않도록 함
        pen_req = SetPen.Request()
        pen_req.off = 1  # 펜 비활성화 (off)
        
        future = self.set_pen_client.call_async(pen_req)
        future.add_done_callback(self._pen_up_done_callback)

    def _pen_up_done_callback(self, future):
        """펜을 든 후 호출되는 콜백. 텔레포트를 실행합니다."""
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(f'펜 들기 서비스 호출 실패: {e}')

        # 2. 시작 지점으로 텔레포트
        teleport_req = TeleportAbsolute.Request()
        teleport_req.x = self.START_POSE.x
        teleport_req.y = self.START_POSE.y
        teleport_req.theta = random.uniform(0, 2 * math.pi) # 시작 시 랜덤 방향
        teleport_future = self.teleport_client.call_async(teleport_req)
        teleport_future.add_done_callback(self._teleport_done_callback)

    def _teleport_done_callback(self, future):
        """텔레포트 완료 후 호출되는 콜백. 미로 그리기를 시작합니다."""
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(f'텔레포트 서비스 호출 실패: {e}')

        # 3. 화면 클리어
        clear_future = self.clear_client.call_async(Empty.Request())
        clear_future.add_done_callback(self._clear_done_callback)

    def _clear_done_callback(self, future):
        """화면 클리어 후 호출되는 콜백. 펜을 내리고 주행을 시작합니다."""
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(f'화면 지우기 서비스 호출 실패: {e}')
        
        # 4. 펜 설정 및 주행 시작
        self._set_turtle1_pen_and_start_running()

    def _set_turtle1_pen_and_start_running(self):
        """turtle1의 펜을 설정하고 주행 상태로 전환합니다."""
        pen_req = SetPen.Request()
        pen_req.off = 0 # 펜 활성화 (on)
        pen_req.r = self.PEN_R
        pen_req.g = self.PEN_G
        pen_req.b = self.PEN_B
        pen_req.width = self.PEN_WIDTH
        self.set_pen_client.call_async(pen_req)

        self.get_logger().info('시작 지점으로 이동 및 화면 클리어 완료. 랜덤 주행을 시작합니다.')
        self.state = TurtleState.RUNNING

    def is_in_goal_area(self):
        """터틀이 목표 영역 내에 있는지 확인합니다."""
        return (self.GOAL_AREA.x_min <= self.current_pose.x <= self.GOAL_AREA.x_max and
                self.GOAL_AREA.y_min <= self.current_pose.y <= self.GOAL_AREA.y_max)

    def is_colliding(self):
        """터틀이 경계 또는 벽에 충돌했는지 확인합니다."""
        margin = self.SAFETY_MARGIN
        pose = self.current_pose

        # 외부 경계 확인
        if not (self.BOUNDARIES.x_min + margin < pose.x < self.BOUNDARIES.x_max - margin and
                self.BOUNDARIES.y_min + margin < pose.y < self.BOUNDARIES.y_max - margin):
            return True
        
        # 내부 벽 확인
        for wall in self.WALLS:
            if (wall.x_min - margin < pose.x < wall.x_max + margin and
                wall.y_min - margin < pose.y < wall.y_max + margin):
                return True
        return False

    def avoid_collision_start(self):
        """충돌 회피 동작을 시작합니다. (정지 및 후진)"""
        # 1. 정지
        self.stop_turtle()
        self.state = TurtleState.AVOIDING_BACKWARD
        # 2. 즉시 후진 시작
        twist_msg = Twist()
        twist_msg.linear.x = -self.BACKWARD_SPEED
        self.cmd_vel_publisher.publish(twist_msg)

    def start_rotation_callback(self):
        """후진 완료 후 호출되는 콜백. 회전을 시작합니다."""
        self.stop_turtle()
        self.state = TurtleState.AVOIDING_ROTATE

        # 90도(pi/2) ~ 270도(3*pi/2) 사이의 랜덤한 회전 각도 생성
        random_rotation = random.uniform(math.pi / 2, 3 * math.pi / 2)

        # 회전 목표 각도 설정
        self.target_theta = normalize_angle(self.current_pose.theta + random_rotation)

    def avoid_collision_rotate(self):
        """목표 각도에 도달할 때까지 회전합니다."""
        # 목표 각도와의 차이 계산
        angle_diff = normalize_angle(self.target_theta - self.current_pose.theta)

        if abs(angle_diff) > 0.1:
            twist_msg = Twist()
            twist_msg.angular.z = self.ANGULAR_SPEED if angle_diff > 0 else -self.ANGULAR_SPEED
            self.cmd_vel_publisher.publish(twist_msg)
        else:
            self.stop_turtle()
            self.get_logger().info('회피 기동 완료. 주행을 재개합니다.')
            self.state = TurtleState.RUNNING

    def move_randomly(self):
        """랜덤한 선속도와 각속도로 주행 명령을 발행합니다."""
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
            node.stop_turtle() # 종료 시 터틀 정지
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
