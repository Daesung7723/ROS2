import sys
import select
import random
import rclpy as rp
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class automove_node:
    def __init__(self):
        self.n_name = input("node name = ")
    
    def Create_node(self):
        self.node = rp.create_node(self.n_name)
    
    def Destroy_node(self):
        self.node.destroy_node()

class automove_topic(automove_node):
    def __init__(self):
        super().__init__()
        super().Create_node()
        self.t_name = input('Turtle name = ')
        self.l = float(input('Low limit = '))
        self.h = float(input('High limit = '))
        self.flag = True
        self.msg = Twist()
        self.b_linear=0.0
        self.b_angular=0.0
        self.create_topic()
    
    def create_topic(self):        
        self.pub = self.node.create_publisher(Twist, '/'+self.t_name+'/cmd_vel', 10)
        self.node.create_subscription(Pose, '/'+self.t_name+'/pose', self.callback, 10)
        self.node.create_timer(0.5, self.timer_callback)

    def callback(self,data) :
        # print(f"%s_Pose> X: %.2f y: %.2f"%(self.t_name,data.x, data.y))
        if ((data.x>self.h)or(data.x<self.l)) or ((data.y>self.h)or(data.y<self.l)): 
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
    
    def run_once(self):
        rp.spin_once(self.node)

def is_key_pushed():    
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

if __name__=='__main__':
    node_list = []
    rp.init()
    while True:
        if is_key_pushed():            
            c = sys.stdin.read(1)            
            if c == 'q': 
                print('byebye')
                break
            elif c == 'a':
                node = automove_topic()
                node_list.append(node)
            elif c == 'r':
                r_name = input('Remove Turtle name :')
                try:
                    for i in range(len(node_list)):
                        if r_name == node_list[i].t_name :
                            node_list[i].Destroy_node()
                            node_list.pop(i)
                except:
                    pass
        for i in range(len(node_list)):
            node_list[i].run_once()
        