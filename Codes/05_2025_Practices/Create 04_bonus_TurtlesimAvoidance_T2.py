import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import random
import math
import time

# 상수 설정
SAFE_BOUNDARY_MIN = 2.0
SAFE_BOUNDARY_MAX = 9.0
CENTER_X = 5.5
CENTER_Y = 5.5
AVOIDANCE_DISTANCE = 1.2

class Turtle2ControllerNode(Node):
    def __init__(self):
        super().__init__('turtle2_controller')
        self.turtle_name = 'turtle2' # 제어 대상: turtle2
        
        # 멤버 변수 초기화
        self.current_pose = None
        self.turtle1_pose = None # turtle1의 위치 (회피 대상)
        self.publisher_ = None
        self.state = 'wandering'

        self.check_and_spawn_turtle()
        
        # 제어를 위한 퍼블리셔, 서브스크라이버, 타이머 생성
        self.publisher_ = self.create_publisher(Twist, f'/{self.turtle_name}/cmd_vel', 10)
        self.self_subscription = self.create_subscription(
            Pose, f'/{self.turtle_name}/pose', self.self_pose_callback, 10)
        # turtle1의 존재와 상관없이 우선 구독 시도
        self.turtle1_subscription = self.create_subscription(
            Pose, '/turtle1/pose', self.turtle1_pose_callback, 10)
        
        timer_period = 0.1
        self.timer_ = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f'"{self.turtle_name}" controller started. Will avoid turtle1 if present.')

    def check_and_spawn_turtle(self):
        spawn_client = self.create_client(Spawn, '/spawn')
        while not spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting...')
        time.sleep(1)
        service_names_and_types = self.get_service_names_and_types()
        turtle2_exists = any(f'/{self.turtle_name}/' in name for name, _ in service_names_and_types)
        if turtle2_exists: self.get_logger().info(f'"{self.turtle_name}" already exists.')
        else:
            self.get_logger().info(f'"{self.turtle_name}" not found. Spawning a new one.')
            request = Spawn.Request()
            request.x, request.y, request.theta, request.name = 8.0, 8.0, 0.0, self.turtle_name
            future = spawn_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result(): self.get_logger().info(f'Successfully spawned "{future.result().name}".')
            else: self.get_logger().error(f'Failed to spawn "{self.turtle_name}".')
        self.destroy_client(spawn_client)

    def self_pose_callback(self, msg: Pose):
        self.current_pose = msg

    def turtle1_pose_callback(self, msg: Pose):
        # turtle1이 나타나면 위치를 업데이트
        self.turtle1_pose = msg

    def timer_callback(self):
        if self.current_pose is None or self.publisher_ is None:
            return
        
        # --- 1. 상태 결정 로직 ---
        is_avoiding = False
        # turtle1의 위치 정보가 있을 때만 회피 로직 검사
        if self.turtle1_pose is not None:
            dist_x = self.current_pose.x - self.turtle1_pose.x
            dist_y = self.current_pose.y - self.turtle1_pose.y
            distance = math.sqrt(dist_x**2 + dist_y**2)
            
            if distance < AVOIDANCE_DISTANCE:
                self.state = 'avoiding'
                is_avoiding = True

        if not is_avoiding:
            is_out_of_bounds = (
                self.current_pose.x > SAFE_BOUNDARY_MAX or self.current_pose.x < SAFE_BOUNDARY_MIN or
                self.current_pose.y > SAFE_BOUNDARY_MAX or self.current_pose.y < SAFE_BOUNDARY_MIN
            )
            if is_out_of_bounds:
                self.state = 'recovering'
            else:
                self.state = 'wandering'
        
        # --- 2. 상태에 따른 행동 결정 ---
        twist_msg = Twist()
        if self.state == 'avoiding':
            self.get_logger().warn('State: Avoiding turtle1!', throttle_duration_sec=1)
            escape_angle = math.atan2(self.current_pose.y - self.turtle1_pose.y, self.current_pose.x - self.turtle1_pose.x)
            angle_error = escape_angle - self.current_pose.theta
            angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
            twist_msg.angular.z = 4.0 * angle_error
            twist_msg.linear.x = 1.5
        elif self.state == 'recovering':
            self.get_logger().info('State: Recovering from wall.', throttle_duration_sec=1)
            dx = CENTER_X - self.current_pose.x
            dy = CENTER_Y - self.current_pose.y
            target_angle = math.atan2(dy, dx)
            angle_error = target_angle - self.current_pose.theta
            angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
            twist_msg.angular.z = 2.5 * angle_error
            twist_msg.linear.x = 1.0
        else: # self.state == 'wandering'
            self.get_logger().info('State: Wandering freely.', throttle_duration_sec=1)
            twist_msg.linear.x = random.uniform(1.5, 3.0)
            twist_msg.angular.z = random.uniform(-2.5, 2.5)
            
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Turtle2ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
