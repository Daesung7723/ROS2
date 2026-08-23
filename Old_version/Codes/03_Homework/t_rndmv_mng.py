import random
import rclpy as rp
from rclpy.node import Node
from std_srvs.srv import Empty
from turtlesim.srv import SetPen, TeleportAbsolute, Spawn, Kill
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from first_msg.srv import TurtleCmd
from hmwk01_msg.msg import HmWk01

class Manager_node(Node):
    def __init__(self):
        super().__init__('mng_node')
        print('01 - NodeCreated')
        self.node_list = []
        self.turt_sts = []
        self.srv = self.create_service(TurtleCmd, 'TurtleCmdSrv', self.callback_tsrv)
        self.timer = self.create_timer(0.01, self.timer_callback)

    def callback_tsrv(self, req, res):
        print('02 - Cmd Rx')
        self.rx_req = req
        print('Rx Req>  ', end='')
        print(req)
        self.handle_cmd()
        return res

    def handle_cmd(self):
        if self.rx_req.cmd == 'set':
            if self.rx_req.name not in self.get_node_names():
                data = HmWk01()
                data.t_name = self.rx_req.name
                data.p_x = self.rx_req.x
                data.p_y = self.rx_req.y
                data.l_x = self.rx_req.x_l
                data.l_y = self.rx_req.y_l
                self.node = AutonomousDrive_node(data)
                self.node_list.append(self.node)
                self.turt_sts.append(False)
            else:
                print('already created.')
        elif self.rx_req.cmd == 'del':
            if self.rx_req.name in self.get_node_names():
                for i in range(len(self.node_list)):
                    if self.rx_req.name == self.node_list[i].t_name :
                        del_node = self.node_list.pop(i)
                        self.turt_sts.pop(i)
                        del_node.destroy_node()                                                
            else:
                print("node is not exist")
        elif self.rx_req.cmd == 'run':
            if self.rx_req.name in self.get_node_names():
                for i in range(len(self.node_list)):
                    if self.rx_req.name == self.node_list[i].t_name :
                        self.turt_sts[i] = True
            else:
                print("node is not exist")
        elif self.rx_req.cmd == 'stop':
            if self.rx_req.name in self.get_node_names():
                for i in range(len(self.node_list)):
                    if self.rx_req.name == self.node_list[i].t_name :
                        self.turt_sts[i] = False
            else:
                print("node is not exist")
        elif self.rx_req.cmd == 'quit': self.destroy_node()
        elif self.rx_req.cmd == 'reset': self.reset()
        elif self.rx_req.cmd == 'clear': self.clear()
        elif self.rx_req.cmd == 'spawn': self.spawn()
        elif self.rx_req.cmd == 'kill': self.kill()
        elif self.rx_req.cmd == 'setpen': self.setpen()
        elif self.rx_req.cmd == 'telabs': self.teleport_abs()

    def timer_callback(self):
        for i in range(len(self.node_list)):
            if self.turt_sts[i] :
                rp.spin_once(self.node_list[i])
        
    def reset(self):
        client_rst = self.create_client(Empty,'/reset')
        rst_req = Empty.Request()
        client_rst.call_async(rst_req)
        client_rst.destroy()
        print('Tx Req>  ', end='')
        print(rst_req)

    def clear(self):
        client_clr = self.create_client(Empty,'/clear')
        clr_req = Empty.Request()
        client_clr.call_async(clr_req)
        client_clr.destroy()
        print('Tx Req>  ', end='')
        print(clr_req)

    def spawn(self):
        client_spawn = self.create_client(Spawn, '/spawn')
        spawn_req = Spawn.Request()
        spawn_req.name = self.rx_req.name
        spawn_req.x = self.rx_req.x
        spawn_req.y = self.rx_req.y
        spawn_req.theta = self.rx_req.theta
        client_spawn.call_async(spawn_req)
        client_spawn.destroy()
        print('Tx Req>  ', end='')
        print(spawn_req)

    def kill(self):
        client_kill = self .create_client(Kill, '/kill')
        kill_req = Kill.Request()
        kill_req.name = self.rx_req.name
        client_kill.call_async(kill_req)
        client_kill.destroy()
        print('Tx Req>  ', end='')
        print(kill_req)

    def setpen(self):
        t_name = self.rx_req.name
        client_setpen = self.create_client(SetPen, '/'+t_name+'/set_pen')    
        setpen_req = SetPen.Request()
        setpen_req.r = self.rx_req.r
        setpen_req.g = self.rx_req.g
        setpen_req.b = self.rx_req.b
        setpen_req.width = self.rx_req.w
        client_setpen.call_async(setpen_req)
        client_setpen.destroy()
        print('Tx Req>  ', end='')
        print(setpen_req)

    def teleport_abs(self):
        t_name = self.rx_req.name
        client_tabs = self.create_client(TeleportAbsolute, '/'+t_name+'/teleport_absolute')    
        tabs_req = TeleportAbsolute.Request()
        tabs_req.x = self.rx_req.x
        tabs_req.y = self.rx_req.y
        tabs_req.theta = self.rx_req.theta
        client_tabs.call_async(tabs_req)
        client_tabs.destroy()
        print('Tx Req>  ', end='')
        print(tabs_req)


class AutonomousDrive_node(Node):
    def __init__(self, cmd): 
        super().__init__(cmd.t_name)
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
        