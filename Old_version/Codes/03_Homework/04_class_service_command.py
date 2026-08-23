import rclpy as rp
from std_srvs.srv import Empty
from turtlesim.srv import SetPen, TeleportAbsolute, TeleportRelative, Spawn, Kill

class turtlesim_srv_client:
    def reset(node):
        client_rst = node.create_client(Empty,'/reset')
        rst_req = Empty.Request()
        client_rst.call_async(rst_req)
        client_rst.destroy()

    def clear(node):
        client_clr = node.create_client(Empty,'/clear')
        clr_req = Empty.Request()
        client_clr.call_async(clr_req)
        client_clr.destroy()

    def spawn(node):
        client_spawn = node.create_client(Spawn, '/spawn')
        spawn_req = Spawn.Request()
        spawn_req.name = input('Turtle name = ')
        spawn_req.x, spawn_req.y, spawn_req.theta \
            = map(float, input('X Y Theta = ').split())    
        client_spawn.call_async(spawn_req)
        client_spawn.destroy()

    def kill(node):
        client_kill = node.create_client(Kill, '/kill')
        kill_req = Kill.Request()
        kill_req.name = input('Turtle name = ')
        client_kill.call_async(kill_req)
        client_kill.destroy()

    def setpen(node):
        t_name = input('Turtle name = ')
        client_setpen = node.create_client(SetPen, '/'+t_name+'/set_pen')    
        setpen_req = SetPen.Request()
        setpen_req.r, setpen_req.g, setpen_req.b, setpen_req.width \
            = map(int, input('R G B W = ').split())    
        client_setpen.call_async(setpen_req)
        client_setpen.destroy()

    def teleport_abs(node):
        t_name = input('Turtle name = ')
        client_tabs = node.create_client(TeleportAbsolute, '/'+t_name+'/teleport_absolute')    
        tabs_req = TeleportAbsolute.Request()
        tabs_req.x, tabs_req.y, tabs_req.theta \
            = map(float, input('X Y Theta = ').split())    
        client_tabs.call_async(tabs_req)
        client_tabs.destroy()

    def teleport_rel(node):
        t_name = input('Turtle name = ')
        client_trel = node.create_client(TeleportRelative, '/'+t_name+'/teleport_relative')
        trel_req = TeleportRelative.Request()
        trel_req.linear, trel_req.angular \
            = map(float, input('L A = ').split())
        client_trel.call_async(trel_req)    
        client_trel.destroy()

def menu() -> int:
    print('1. reset')
    print('2. clear')
    print('3. spawn')
    print('4. kill')
    print('5. set pen')
    print('6. teleport abs')
    print('7. teleport rel')
    print('8. exit')
    try:
        menu = int(input('Select num = '))
        return menu
    except:
        return 0

if __name__ == '__main__':
    rp.init()
    SrvCli_node = rp.create_node('Sevice_Command')
    SrvCli = turtlesim_srv_client
    
    while True:
        m = menu()
        if m == 8 : break
        elif m==1:  SrvCli.reset(SrvCli_node)
        elif m==2:  SrvCli.clear(SrvCli_node)
        elif m==3:  SrvCli.spawn(SrvCli_node)
        elif m==4:  SrvCli.kill(SrvCli_node)
        elif m==5:  SrvCli.setpen(SrvCli_node)
        elif m==6:  SrvCli.teleport_abs(SrvCli_node)
        elif m==7:  SrvCli.teleport_rel(SrvCli_node)