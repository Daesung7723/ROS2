import rclpy as rp
from rclpy.node import Node
from first_msg.srv import TurtleCmd

class Cmd(Node):
    def __init__(self):
        super().__init__('cmd_node')
        self.req = TurtleCmd.Request()
        self.res = TurtleCmd.Response()
        self.cmd_dic = {'mov':[ 'set', 'del', 'run', 'stop', 'quit'],\
                        'set':['reset', 'clear', 'spawn', 'kill', 'setpen','telabs']}
        self.cli = self.create_client(TurtleCmd, '/TurtleCmdSrv')

    def callback_res(self):
        print(self.future.result())
        
    def set_req(self, cmd):
        self.req.cmd = cmd
        try:
            if cmd in ['reset', 'clear', 'quit']:
                pass
            elif cmd in ['del', 'run', 'stop', 'kill']:
                self.req.name = input('Turtle name = ')
            elif cmd in ['spawn', 'telabs']:
                self.req.name = input('Turtle name = ')
                self.req.x, self.req.y, self.req.theta \
                    =map(float, input('x y theta = ').split())
            elif cmd == 'set':
                self.req.name = input('Turtle name = ')
                self.req.x, self.req.y, self.req.x_l, self.req.y_l\
                    =map(float, input('x y x_len y_len = ').split())
            elif cmd == 'setpen':
                self.req.name = input('Turtle name : ')
                self.req.r, self.req.g, self.req.b, self.req.w\
                    =map(int, input('r g b w = ').split())
        
            self.future = self.cli.call_async(self.req)
        except:
            pass
        
    def run(self):
        try:
            pre_cmd = input('select cmd  ->'+str(self.cmd_dic.keys()) +'= ')
            if pre_cmd in self.cmd_dic.keys():
                tmp = input('selectt cmd ->'+str(self.cmd_dic[pre_cmd])+'= ')
                if tmp in self.cmd_dic[pre_cmd]:
                    cmd = tmp
                else: pass
            else:
                pass
        
            self.set_req(cmd)
        except:
            pass

def main(args=None):
    rp.init(args=args)

    cmd = Cmd()
    while True:
        cmd.run()
        if cmd.req.cmd == 'quit':
            break

if __name__=='__main__':
    main()