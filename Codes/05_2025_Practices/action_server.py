import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import math
import time
import random

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute

from act_msg.action import MoveInGoal

# -------------------------------------------------------------------
# 동작 로직 계산 클래스
# -------------------------------------------------------------------
class TurtleBehaviors:
    """
    거북이의 개별 동작 로직을 계산하는 헬퍼 클래스.
    """
    def is_near_boundary(self, current_pose: Pose, min_x, max_x, min_y, max_y, padding=0.5) -> bool:
        """지정된 경계에 가까운지 확인합니다."""
        return (current_pose.x < min_x + padding or
                current_pose.x > max_x - padding or
                current_pose.y < min_y + padding or
                current_pose.y > max_y - padding)

    def get_turn_to_target_twist(self, current_pose: Pose, target_x: float, target_y: float) -> (Twist, bool):
        """목표 지점을 바라보도록 회전하는 Twist 메시지를 생성합니다."""
        twist_msg = Twist()
        is_done = False
        target_angle = math.atan2(target_y - current_pose.y, target_x - current_pose.x)
        angle_diff = math.atan2(math.sin(target_angle - current_pose.theta),
                                math.cos(target_angle - current_pose.theta))

        if abs(angle_diff) > 0.1:
            twist_msg.angular.z = 4.0 * angle_diff
        else:
            is_done = True
            twist_msg.angular.z = 0.0
        return twist_msg, is_done
    
    def get_random_drive_twist(self) -> Twist:
        """랜덤 주행을 위한 Twist 메시지를 생성합니다."""
        twist_msg = Twist()
        twist_msg.linear.x = random.uniform(1.5, 2.5)
        twist_msg.angular.z = random.uniform(-3.0, 3.0)
        return twist_msg

    def get_reverse_twist(self) -> Twist:
        """후진을 위한 Twist 메시지를 생성합니다."""
        twist_msg = Twist()
        twist_msg.linear.x = -1.0
        return twist_msg

# -------------------------------------------------------------------
# Action Server 노드 클래스
# -------------------------------------------------------------------
class RectangleActionServer(Node):
    def __init__(self):
        super().__init__('rectangle_action_server')
        self.current_pose_ = None
        self.is_pose_ready_ = False
        self.behaviors = TurtleBehaviors()
        
        self.callback_group = ReentrantCallbackGroup()
        
        self.pose_subscriber_ = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10, callback_group=self.callback_group)
        self.cmd_vel_publisher_ = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)
        self.clear_client_ = self.create_client(Empty, '/clear', callback_group=self.callback_group)
        self.teleport_client_ = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute', callback_group=self.callback_group)
        
        self.action_server_ = ActionServer(
            self, MoveInGoal, 'move_in_rectangle',
            execute_callback=self.execute_callback,
            goal_callback=lambda gr: GoalResponse.ACCEPT,
            handle_accepted_callback=lambda gh: gh.execute(),
            cancel_callback=lambda cr: CancelResponse.ACCEPT,
            callback_group=self.callback_group)

        self.wait_for_services()
        self.get_logger().info("Action Server has been started and is ready for goals.")

    def pose_callback(self, msg):
        self.current_pose_ = msg
        self.is_pose_ready_ = True

    def wait_for_services(self):
        """노드 시작 시 필요한 서비스들이 활성화될 때까지 대기합니다."""
        while not self.clear_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/clear service not available, waiting again...')
        while not self.teleport_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/turtle1/teleport_absolute service not available, waiting again...')
        self.get_logger().info("All required services are active.")

    def execute_callback(self, goal_handle):
        self.get_logger().info('New goal received! Resetting simulation state...')

        # 1. 터틀을 초기 위치로 이동하고 화면을 클리어
        teleport_req = TeleportAbsolute.Request(x=2.0, y=2.0, theta=0.78)
        self.teleport_client_.call_async(teleport_req)
        self.clear_client_.call_async(Empty.Request())

        # 서비스 호출이 완료될 때까지 잠시 대기
        time.sleep(1.0)
        self.get_logger().info("Reset complete. Starting random drive from initial position...")
        
        goal = goal_handle.request
        
        # 절대 이동 영역 설정
        ABS_MIN_X, ABS_MAX_X = 1.0, 10.0
        ABS_MIN_Y, ABS_MAX_Y = 1.0, 10.0
        ABS_CENTER_X = (ABS_MIN_X + ABS_MAX_X) / 2.0
        ABS_CENTER_Y = (ABS_MIN_Y + ABS_MAX_Y) / 2.0
        
        # 상태 머신 변수
        drive_state = 'DRIVING'
        state_counter = 0

        # 메인 실행 루프
        while rclpy.ok():
            if not self.is_pose_ready_:
                time.sleep(0.1)
                continue

            if goal_handle.is_cancel_requested:
                return self.handle_cancel(goal_handle)

            if self.is_in_destination_area(self.current_pose_, goal):
                self.get_logger().info("Destination area reached! Goal succeeded.")
                self.stop_turtle()
                goal_handle.succeed()
                return MoveInGoal.Result(success=True)

            dist_to_dest_center = math.sqrt((goal.x - self.current_pose_.x)**2 + (goal.y - self.current_pose_.y)**2)
            goal_handle.publish_feedback(MoveInGoal.Feedback(remaining_distance=dist_to_dest_center))

            is_near_abs_boundary = self.behaviors.is_near_boundary(
                self.current_pose_, ABS_MIN_X, ABS_MAX_X, ABS_MIN_Y, ABS_MAX_Y)
            
            twist_msg = Twist()
            
            if drive_state == 'DRIVING':
                if is_near_abs_boundary:
                    drive_state, state_counter = 'REVERSING', 5
                    twist_msg = self.behaviors.get_reverse_twist()
                else:
                    twist_msg = self.behaviors.get_random_drive_twist()
            elif drive_state == 'REVERSING':
                twist_msg = self.behaviors.get_reverse_twist()
                state_counter -= 1
                if state_counter <= 0:
                    drive_state = 'TURNING'
            elif drive_state == 'TURNING':
                twist_msg, is_done = self.behaviors.get_turn_to_target_twist(
                    self.current_pose_, ABS_CENTER_X, ABS_CENTER_Y)
                if is_done:
                    drive_state = 'DRIVING'

            self.cmd_vel_publisher_.publish(twist_msg)
            time.sleep(0.1)

        goal_handle.abort()
        return MoveInGoal.Result(success=False)

    def is_in_destination_area(self, pose, goal):
        return (goal.x <= pose.x <= goal.x + goal.x_length and
                goal.y <= pose.y <= goal.y + goal.y_length)

    def handle_cancel(self, goal_handle):
        goal_handle.canceled()
        self.stop_turtle()
        self.get_logger().info('Goal canceled!')
        return MoveInGoal.Result(success=False)

    def stop_turtle(self):
        self.cmd_vel_publisher_.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    action_server = RectangleActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

