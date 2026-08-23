import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import random
import math

# 안전 경계 및 화면 중앙 설정
SAFE_BOUNDARY_MIN = 2.0
SAFE_BOUNDARY_MAX = 9.0
CENTER_X = 5.5
CENTER_Y = 5.5

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__('turtle_controller_node')
        self.get_logger().info('Turtle Controller Node has been started.')

        self.is_wandering = True
        self.current_pose = None  # 현재 거북이의 위치와 방향을 저장할 변수

        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription_ = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        timer_period = 0.2  # 반응 속도를 높이기 위해 타이머 주기를 줄임
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

    def pose_callback(self, msg: Pose):
        """
        거북이의 위치를 계속 업데이트하고, 안전 경계 이탈 여부에 따라 상태를 변경
        """
        self.current_pose = msg
        
        is_out_of_safe_bounds = (
            msg.x > SAFE_BOUNDARY_MAX or msg.x < SAFE_BOUNDARY_MIN or
            msg.y > SAFE_BOUNDARY_MAX or msg.y < SAFE_BOUNDARY_MIN
        )

        if is_out_of_safe_bounds:
            if self.is_wandering:
                self.get_logger().warn(f'Safe boundary breached at X:{msg.x:.2f}, Y:{msg.y:.2f}. Starting recovery.')
                self.is_wandering = False # 복귀 모드 시작
        else:
            if not self.is_wandering:
                self.get_logger().info('Back in safe area. Resuming wandering.')
                self.is_wandering = True # 주행 모드 시작

    def timer_callback(self):
        """
        현재 상태(is_wandering)에 따라 무작위 주행 또는 중앙 복귀 동작을 수행
        """
        if self.current_pose is None:
            # 아직 위치 정보를 받지 못했다면 아무것도 하지 않음
            return

        twist_msg = Twist()

        if self.is_wandering:
            # [주행 모드] 무작위 선속도와 각속도 생성
            twist_msg.linear.x = random.uniform(1.5, 3.0)
            twist_msg.angular.z = random.uniform(-2.5, 2.5)
        else:
            # [복귀 모드] 화면 중앙(CENTER_X, CENTER_Y)을 향하도록 제어
            dx = CENTER_X - self.current_pose.x
            dy = CENTER_Y - self.current_pose.y
            
            # 목표 각도와 현재 각도의 차이 계산
            target_angle = math.atan2(dy, dx)
            angle_error = target_angle - self.current_pose.theta
            
            # -pi ~ +pi 범위로 정규화
            if angle_error > math.pi:
                angle_error -= 2 * math.pi
            elif angle_error < -math.pi:
                angle_error += 2 * math.pi

            # 중앙을 향해 회전하고, 동시에 느리게 전진
            twist_msg.angular.z = 2.0 * angle_error # 오차에 비례해 회전 속도 조절
            twist_msg.linear.x = 0.8 # 벽에서 벗어나기 위한 최소한의 전진 속도
        
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly by user (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
