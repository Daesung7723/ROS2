import rclpy
from rclpy.node import Node
import random
import math
import sys

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleDriverNode(Node):
    """
    개별 거북이의 랜덤 자율 주행을 담당하는 독립 노드
    """
    def __init__(self, turtle_name):
        super().__init__(f'{turtle_name}_driver_node')
        self.turtle_name = turtle_name

        # 현재 위치 저장을 위한 변수
        self.current_pose = None

        # 자율 주행 상태 변수
        self.drive_state = 'DRIVING'
        self.state_counter = 0

        # 퍼블리셔, 서브스크라이버, 타이머 생성
        self.cmd_vel_publisher = self.create_publisher(
            Twist, f'/{self.turtle_name}/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(
            Pose, f'/{self.turtle_name}/pose', self.pose_callback, 10)
        self.drive_timer = self.create_timer(0.1, self.drive_callback)

        self.get_logger().info(f"'{self.turtle_name}'의 Driver Node가 시작되었습니다.")

    def pose_callback(self, msg: Pose):
        self.current_pose = msg

    def drive_callback(self):
        if self.current_pose is None:
            return

        is_near_boundary = (self.current_pose.x < 1.5 or self.current_pose.x > 9.5 or
                            self.current_pose.y < 1.5 or self.current_pose.y > 9.5)

        twist_msg = Twist()

        if self.drive_state == 'DRIVING':
            if is_near_boundary:
                self.drive_state = 'REVERSING'
                self.state_counter = 2
                twist_msg.linear.x = -1.5
            else:
                twist_msg.linear.x = random.uniform(1.0, 2.0)
                twist_msg.angular.z = random.uniform(-3.0, 3.0)

        elif self.drive_state == 'REVERSING':
            twist_msg.linear.x = -1.5
            self.state_counter -= 1
            if self.state_counter <= 0:
                self.drive_state = 'TURNING_TO_CENTER'

        elif self.drive_state == 'TURNING_TO_CENTER':
            target_angle = math.atan2(5.5 - self.current_pose.y, 5.5 - self.current_pose.x)
            angle_diff = math.atan2(math.sin(target_angle - self.current_pose.theta), math.cos(target_angle - self.current_pose.theta))

            if abs(angle_diff) > 0.1:
                twist_msg.angular.z = 2.0 * angle_diff
            else:
                self.drive_state = 'DRIVING'

        self.cmd_vel_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)

    # 커맨드 라인 인자로부터 거북이 이름을 받습니다.
    if len(sys.argv) < 2:
        print("에러: 거북이 이름이 인자로 필요합니다.")
        return

    turtle_name = sys.argv[1]
    driver_node = TurtleDriverNode(turtle_name)

    try:
        rclpy.spin(driver_node)
    except KeyboardInterrupt:
        pass
    finally:
        driver_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
