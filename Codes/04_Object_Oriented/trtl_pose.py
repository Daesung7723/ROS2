import rclpy as rp
from rclpy.node import Node
from turtlesim.msg import Pose

class Trtl_pose(Node):
    def __init__(self, n_name='trtl_pose', t_name='turtle1', prt=False):
        super().__init__(n_name)
        self.prt = prt
        self.sub = self.create_subscription(Pose, '/'+ t_name +'/pose', self.sub_callback, 10)

    def sub_callback(self, pos_msg):
        self.px = pos_msg.x
        self.py = pos_msg.y
        if self.prt :
            print('X : %.2f Y: %.2f'%(self.px, self.py))

def main(args=None):
    rp.init(args=args)

    trtlmv = Trtl_pose(prt=True)
    rp.spin(trtlmv)

    trtlmv.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()