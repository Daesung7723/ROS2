import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseRelayNode(Node):
    """
    /turtle1/pose 토픽을 구독하여 터미널에 출력하고, 
    /T1_Pose 라는 새 이름으로 다시 발행하는 노드.
    """
    def __init__(self):
        super().__init__('pose_relay_node')
        
        self.publisher_ = self.create_publisher(Pose, '/T1_Pose', 10)
        self.subscription_ = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.relay_callback,
            10)
            
        self.get_logger().info('Pose Relay Node가 시작되었습니다. /turtle1/pose -> /T1_Pose')

    def relay_callback(self, msg: Pose):
        """
        /turtle1/pose 로부터 메시지를 받으면 터미널에 출력하고 다시 발행합니다.
        """
        # 수신된 메시지의 x, y, theta 값을 터미널에 정보(INFO) 로그로 출력합니다.
        self.get_logger().info(f'수신 데이터 -> X: {msg.x:.2f}, Y: {msg.y:.2f}, Theta: {msg.theta:.2f}')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PoseRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
