import random
import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from hmwk01_msg.msg import HmWk01

class Manager_node(Node):
    def __init__(self):
        super().__init__('mng_node')
        self.msg = HmWk01()
        self.sub = self.create_subscription(HmWk01, '/HmWk01', self.callback_cmd, 10)
        print('01 - NodeCreated')

    def timer_callback(self):
        rp.spin_once(self.node)

    def callback_cmd(self, data):
        print('02 - Cmd Rx')
        self.msg = data
        print(data)

        if data.cmd == 'set':
            if 'rnd_mv_node' not in self.get_node_names():
                self.node = AutonomousDrive_node(data)
            else:
                print('already created.')
        elif data.cmd == 'del':
            if 'rnd_mv_node' in self.get_node_names():
                self.node.destroy_node()
            else:
                print("node is not exist")
        elif data.cmd == 'run':
            if 'rnd_mv_node' in self.get_node_names():
                self.timer = self.create_timer(0.005, self.timer_callback)
            else:
                print("node is not exist")
        elif data.cmd == 'stop':
            if 'rnd_mv_node' in self.get_node_names():
                self.timer.destroy()
            else:
                print("node is not exist")
        elif data.cmd == 'quit':
            self.destroy_node()


class AutonomousDrive_node(Node):
    def __init__(self, cmd): 
        super().__init__('rnd_mv_node')
        self.t_name = cmd.t_name
        self.p_x = cmd.p_x
        self.p_y = cmd.p_y
        self.l_x = cmd.l_x
        self.l_y = cmd.l_y
        self.flag = True
        self.msg = Twist()
        self.b_linear=0.0
        self.b_angular=0.0
        self.pub = self.create_publisher(Twist, '/'+self.t_name+'/cmd_vel', 10)
        self.create_timer(0.5, self.timer_callback)
        self.create_subscription(Pose, '/'+self.t_name+'/pose', self.callback_outrange, 10)

    def callback_outrange(self, data) :
        if ((data.x>(self.p_x+self.l_x))or(data.x<self.p_x)) \
            or ((data.y>(self.p_y+self.l_y))or(data.y<self.p_y)): 
            self.flag = False
            self.msg.linear.x = -self.b_linear
            self.msg.angular.z = -self.b_angular
            self.pub.publish(self.msg)
            self.msg.linear.x = 0.0
            self.msg.angular.z = 3.14
            self.pub.publish(self.msg)
        else : 
            self.flag = True
    
    def timer_callback(self):
        if self.flag :
            rnd_sign = random.randint(0,1)
            rnd_linear = float(random.randint(10,30))/10.0
            rnd_angle = float(random.randint(1,314))/100.0

            self.b_linear = rnd_linear
            self.b_angular = rnd_angle
            
            self.msg.linear.x = rnd_linear
            if(rnd_sign) : self.msg.angular.z = rnd_angle
            else : self.msg.angular.z = -rnd_angle
    
            self.pub.publish(self.msg)

def main(args=None):
    rp.init(args=args)

    mng = Manager_node()
    rp.spin(mng)    

    mng.destroy_node()
    rp.shutdown()

if __name__=='__main__':
    main()
        