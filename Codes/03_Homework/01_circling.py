import  rclpy as rp
from geometry_msgs.msg import Twist

rp.init()
m1_node = rp.create_node('pub_circling')
msg = Twist()

pub = m1_node.create_publisher(Twist, '/turtle1/cmd_vel', 10)

msg.linear.x = 2.0
msg.angular.z = 3.14

def timer_callback():
    pub.publish(msg)

timer_period = 1
timer = m1_node.create_timer(timer_period, timer_callback)
rp.spin(m1_node)