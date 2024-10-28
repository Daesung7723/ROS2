import random
import rclpy as rp
from trtl_pkg.trtl_move import Trtl_move

class Trtl_rnd_move(Trtl_move):
    def __init__(self, n_name='trtl_rnd_move', t_name='turtle1'):
        super().__init__(n_name=n_name, t_name=t_name)
        self.b_linear = 0.
        self.b_angular = 0.
    
    def ftrtl_move(self):
        self.ftrtl_rnd_move()
        self.pub.publish(self.mov_msg)
    
    def ftrtl_rnd_move(self):
        rnd_sign = random.randint(0,1)
        rnd_linear = float(random.randint(10,30))/10.0
        rnd_angle = float(random.randint(1,314))/100.0

        self.b_linear = rnd_linear
        self.b_angular = rnd_angle
        
        self.mov_msg.linear.x = rnd_linear
        if(rnd_sign) : self.mov_msg.angular.z = rnd_angle
        else : self.mov_msg.angular.z = -rnd_angle

def main(args=None):
    rp.init(args=args)

    trtl_rnd_mv = Trtl_rnd_move()
    rp.spin(trtl_rnd_mv)

    trtl_rnd_mv.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()