import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Trtl_move(Node):
    def __init__(self, n_name='trtl_move', t_name='turtle1'):
        super().__init__(n_name)
        self.l = 1.0
        self.a = 1.0
        self.tspan = 0.5
        self.mov_msg = Twist()
        self.pub = self.create_publisher(Twist,'/'+ t_name +'/cmd_vel', 10)
        self.timer = self.create_timer(self.tspan, self.pub_timer_callback)

    def pub_timer_callback(self):
        self.ftrtl_move()
    
    def ftrtl_move(self):
        self.mov_msg.linear.x = self.l
        self.mov_msg.angular.z = self.a
        self.pub.publish(self.mov_msg)

def main(args=None):
    rp.init(args=args)

    trtlmv = Trtl_move()
    rp.spin(trtlmv)

    trtlmv.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()