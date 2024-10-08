import random
import rclpy as rp
from rclpy.node import Node
from hmwk01_msg.msg import HmWk01

class Cmd(Node):
    def __init__(self):
        super().__init__('cmd_node')
        self.msg = HmWk01()
        self.cmd_list = ['set', 'del', 'run', 'stop', 'quit']
        self.pub = self.create_publisher(HmWk01, '/HmWk01', 10)

    def run(self):
        self.msg.cmd = input('chosse cmd '+ str(self.cmd_list) +'= ')
        
        try :
            if self.msg.cmd == 'set':
                self.msg.t_name = input('Turtle_name = ') 
                self.msg.p_x, self.msg.p_y, self.msg.l_x, self.msg.l_y \
                    = map(float, input('x y x_len y_len = ').split())
            
            if self.msg.cmd  in self.cmd_list:
                self.pub.publish(self.msg)
                print('send> ', end='')
                print(self.msg)
        except:
            pass

def main(args=None):
    rp.init(args=args)

    cmd = Cmd()
    while True:
        cmd.run()
        if cmd.msg.cmd == 'quit':
            break

if __name__=='__main__':
    main()
        