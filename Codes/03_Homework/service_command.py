import rclpy as rp
from std_srvs.srv import Empty
from turtlesim.srv import SetPen, TeleportAbsolute, TeleportRelative

rp.init()
m3_node = rp.create_node('Sevice_Command')

client_clr = m3_node.create_client(Empty,'/clear')
client_rst = m3_node.create_client(Empty,'/reset')
client_setpen = m3_node.create_client(SetPen, '/turtle1/set_pen')
client_tabs = m3_node.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
client_trel = m3_node.create_client(TeleportRelative, '/turtle1/teleport_relative')


def menu() -> int:
    print('1. reset')
    print('2. clear')
    print('3. set pen')
    print('4. teleport abs')
    print('5. teleport rel')
    print('6. exit')
    try:
        menu = int(input('Select num = '))
        return menu
    except:
        return 0

def reset():
    rst_req = Empty.Request()
    client_rst.call_async(rst_req)

def clear():
    clr_req = Empty.Request()
    client_clr.call_async(clr_req)

def setpen():
    setpen_req = SetPen.Request()

    setpen_req.r, setpen_req.g, setpen_req.b, setpen_req.width \
        = map(int, input('R G B W = ').split())
    
    client_setpen.call_async(setpen_req)

def teleport_abs():
    tabs_req = TeleportAbsolute.Request()

    tabs_req.x, tabs_req.y, tabs_req.theta \
        = map(float, input('X Y Theta = ').split())
    client_tabs.call_async(tabs_req)

def teleport_rel():
    trel_req = TeleportRelative.Request()
    trel_req.linear, trel_req.angular \
        = map(float, input('L A = ').split())
    client_trel.call_async(trel_req)

def main():
    while True:
        m = menu()
        if m == 6 : break
        elif m==1:
            reset()
        elif m==2:
            clear()
        elif m==3:
            setpen()
        elif m==4:
            teleport_abs()
        elif m==5:
            teleport_rel()

if __name__ == '__main__':
    main()