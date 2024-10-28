import rclpy as rp
from dataclasses import dataclass
from trtl_pkg.trtl_rnd_move import Trtl_rnd_move 
from trtl_pkg.trtl_pose import Trtl_pose 

@dataclass
class Area_data:
    px : float = None
    py : float = None
    xl : float = None
    yl : float = None

class Trtl_bounded_rndmov(Trtl_rnd_move, Trtl_pose):
    def __init__(self, n_name='trtl_bounded_rndmov', t_name='turtle1'):
        super().__init__(n_name=n_name, t_name=t_name)
        self.flag = True
        self.bnd_data = Area_data()
        (self.bnd_data.px, self.bnd_data.py, self.bnd_data.xl, self.bnd_data.yl) \
            = (0.5, 0.5, 10.5, 10.5)

    def sub_callback(self, pos_msg):
        self.px = pos_msg.x
        self.py = pos_msg.y
        if self.prt :
            print('X : %.2f Y: %.2f'%(self.px, self.py))
        self.fcrash_bound()
        self.fcrash_obstacles()
        self.fteleport_point()

    def pub_timer_callback(self):
        if self.flag:
            self.ftrtl_move()
    
    def fcrash_obstacles():
        pass

    def fteleport_point():
        pass

    def fcrash_bound(self):
        if (self.px>(self.bx + self.bxl))or(self.px < self.bx) \
            or (self.py > (self.by + self.byl))or(self.py < self.by) :        
            self.flag = False
            self.favoid_wall()
        else : 
            self.flag = True
    
    def favoid_wall(self):
        self.mov_msg.linear.x  = -self.b_linear
        self.mov_msg.angular.z = -self.b_angular
        self.pub.publish(self.mov_msg)
        self.mov_msg.linear.x = 0.0
        self.mov_msg.angular.z = 3.14
        self.pub.publish(self.mov_msg)

def main(args=None):
    rp.init(args=args)

    trtl_bounded_rndmv = Trtl_bounded_rndmov()
    rp.spin(trtl_bounded_rndmv)

    trtl_bounded_rndmv.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()