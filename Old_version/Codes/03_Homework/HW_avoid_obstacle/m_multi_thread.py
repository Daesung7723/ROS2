import rclpy as rp
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer
from rclpy.node import Node
from turtlesim.msg import Pose
from hmwk01_msg.msg import HmWk01
from first_msg.action import MActionData
from first_pkg.t_rndmv_mng import AutonomousDrive_node
import random

obs_data = [ 
    { 'x':2., 'y':0.5, 'xl':2., 'yl':7.5},
    { 'x':6., 'y':5., 'xl':2, 'yl':5.0},
    { 'x':6., 'y':0.5, 'xl':2., 'yl':3.5}
]

class ActSrv(Node):
    def __init__(self, t_node):
        super().__init__('mMulti_Act_Srv_node')
        self.t_node = t_node
        self.action_server = ActionServer(
            self,
            MActionData,
            'mission_action',
            self.execute_callback)
    
    def execute_callback(self, goal_handle):
        self.goal = HmWk01()
        self.req = goal_handle.request
        feedback_msg = MActionData.Feedback()
        result = MActionData.Result()
        
        self.t_node.goal_x = self.req.p_x
        self.t_node.goal_y = self.req.p_y
        self.t_node.goal_xl = self.req.x_l
        self.t_node.goal_yl = self.req.y_l
        self.t_node.start = True
        self.t_node.goal = False

        feedback_msg = MActionData.Feedback()
        while not self.t_node.goal :
            rp.spin_once(self.t_node)        
            feedback_msg.pose_x = self.t_node.pose.x
            feedback_msg.pose_y = self.t_node.pose.y
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result.rslt = True
        return result

class Autonomous(AutonomousDrive_node):
    def __init__(self, cmd):
        super().__init__(cmd)
        self.goal = False
        self.goal_x =0.
        self.goal_y =0.
        self.goal_xl=0.
        self.goal_yl=0.
        self.create_subscription(Pose, '/'+self.t_name+'/pose', self.callback_obs, 10)
    
    def callback_obs(self, data) :
        self.pose = data

        if ((data.x < (self.goal_xl+self.goal_x))and(data.x > self.goal_x)) \
            and ((data.y < (self.goal_y+self.goal_yl))and(data.y > self.goal_y)): 
            self.start = False
            self.goal = True
            
        for i in range(len(obs_data)):
            if ((data.x<(obs_data[i]['x']+obs_data[i]['xl']))and(data.x>obs_data[i]['x'])) \
                and ((data.y<(obs_data[i]['y']+obs_data[i]['yl']))and(data.y>obs_data[i]['y'])): 
                self.flag = False
                self.avoid_wall()
            else : 
                self.flag = True

    def avoid_wall(self):
        rnd_sign = random.randint(0,1)
        self.msg.linear.x = -self.b_linear
        self.msg.angular.z = -self.b_angular
        self.pub.publish(self.msg)
        self.msg.linear.x = 0.0
        if(rnd_sign) : self.msg.angular.z = 1.57
        else : self.msg.angular.z = -1.57
        self.pub.publish(self.msg)

    def random_move(self):
        rnd_sign = random.randint(0,1)
        rnd_linear = float(random.randint(10,30))/10.0
        rnd_angle = float(random.randint(1,157))/100.0

        self.b_linear = rnd_linear
        self.b_angular = rnd_angle
        
        self.msg.linear.x = rnd_linear
        if(rnd_sign) : self.msg.angular.z = rnd_angle
        else : self.msg.angular.z = -rnd_angle

        self.pub.publish(self.msg)


def main(args=None):
    rp.init(args=args)

    area = HmWk01()
    area.t_name = 'turtle1'
    (area.p_x, area.p_y, area.l_x, area.l_y ) = (0.5, 0.5 , 9.5, 9.5) 
    auto = Autonomous(area)
    actsrv = ActSrv(auto)
    exe = MultiThreadedExecutor()

    exe.add_node(auto)
    exe.add_node(actsrv)

    try:
        exe.spin()
    finally:
        exe.shutdown()
        auto.destroy_node()
        actsrv.destroy_node()
        rp.shutdown()

if __name__ == '__main__':
    main()