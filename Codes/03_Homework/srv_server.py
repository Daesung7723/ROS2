import random
import rclpy as rp
from rclpy.node import Node
from first_msg.srv import MultiSpawn
from turtlesim.srv import Spawn

class Multi_spawn(Node):
    def __init__(self):
        super().__init__('Multi_spawn_node')
        self.turtle_cnt = 1
        self.server = self.create_service(MultiSpawn, 'Multi_spawn', self.callback_srv_server)
        self.req_spawn = Spawn.Request()
        self.cli_spawn = self.create_client(Spawn, '/spawn')

    def callback_srv_server(self, req, res):
        print(req)
        for i in range(req.num):
            res.x.append(float(random.randint(1,10)))
            res.y.append(float(random.randint(1,10)))
            res.theta.append(random.randint(1,314)/100)
            self.req_spawn.x = res.x[i]
            self.req_spawn.y = res.y[i]
            self.req_spawn.theta = res.theta[i]
            self.turtle_cnt += 1
            self.req_spawn.name = str('t'+str(self.turtle_cnt))
            self.cli_spawn.call_async(self.req_spawn)
        return res

def main(args=None):
    rp.init(args=args)
    mlt_spn = Multi_spawn()
    rp.spin(mlt_spn)
    rp.shutdown()

if __name__ == '__main__':
    main()
