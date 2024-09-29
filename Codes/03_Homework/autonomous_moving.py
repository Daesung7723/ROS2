import random
import rclpy as rp
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

flag = True
rp.init()
m2_node = rp.create_node('AmazingNode')
msg = Twist()
b_linear=0.0
b_angular=0.0

def callback(data) :
    global flag    
    print(f"Pose> X: %.2f y: %.2f"%(data.x, data.y))
    if ((data.x>10.0)or(data.x<1.0)) or ((data.y>10.0)or(data.y<1.0)): 
        flag = False
        msg.linear.x = -b_linear
        msg.angular.z = -b_angular
        pub.publish(msg)
        msg.linear.x = 0.0
        msg.angular.z = 3.14
        pub.publish(msg)       
    else : 
        flag = True

def timer_callback() :
    global b_linear
    global b_angular
    if flag :
        rnd_sign = random.randint(0,1)
        rnd_linear = float(random.randint(10,30))/10.0
        rnd_angle = float(random.randint(1,314))/100.0

        b_linear = rnd_linear
        b_angular = rnd_angle
        msg.linear.x = rnd_linear
        if(rnd_sign) : msg.angular.z = rnd_angle
        else : msg.angular.z = -rnd_angle
    
        pub.publish(msg)
    

pub = m2_node.create_publisher(Twist, '/turtle1/cmd_vel', 10)
sub = m2_node.create_subscription(Pose, '/turtle1/pose', callback, 10)

timer_period = 0.5
timer = m2_node.create_timer(timer_period, timer_callback)
rp.spin(m2_node)