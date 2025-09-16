import rclpy
from rclpy.node import Node
from turtlesim.srv import Kill, SetPen
from std_srvs.srv import Empty
import time

class TurtleManagerClientNode(Node):
    def __init__(self):
        super().__init__('turtle_manager_client')
        # 공통적으로 사용할 서비스 클라이언트 미리 생성
        self.kill_client = self.create_client(Kill, '/kill')
        self.clear_client = self.create_client(Empty, '/clear')

    def get_turtle_list(self):
        """
        현재 활성화된 모든 터틀심 거북이의 이름을 리스트로 반환합니다.
        set_pen 서비스의 존재 여부로 거북이를 식별합니다.
        """
        self.get_logger().info('Searching for active turtles...')
        service_names_and_types = self.get_service_names_and_types()
        turtles = []
        for service_name, service_types in service_names_and_types:
            if 'turtlesim/srv/SetPen' in service_types:
                # 서비스 이름 형식: /<turtle_name>/set_pen
                turtle_name = service_name.split('/')[1]
                turtles.append(turtle_name)
        return turtles

    def kill_turtle(self, turtle_name):
        """특정 이름의 거북이를 제거하는 서비스 요청을 보냅니다."""
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Kill service not available, waiting...')
        
        request = Kill.Request()
        request.name = turtle_name
        future = self.kill_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'Turtle "{turtle_name}" has been killed.')

    def handle_set_pen(self, turtle_name):
        """사용자로부터 입력받아 특정 거북이의 펜 설정을 변경합니다."""
        # SetPen 클라이언트는 특정 터틀에 종속적이므로 동적으로 생성
        set_pen_client = self.create_client(SetPen, f'/{turtle_name}/set_pen')
        while not set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'SetPen service for {turtle_name} not ready, waiting...')
        
        # 사용자 입력 및 검증
        while True:
            try:
                r = int(input("  - R (0-255): "))
                g = int(input("  - G (0-255): "))
                b = int(input("  - B (0-255): "))
                width = int(input("  - 두께 (1-255): "))
                off = int(input("  - 선 표시 끄기 (0: 켜기, 1: 끄기): "))
                
                if not (0 <= r <= 255 and 0 <= g <= 255 and 0 <= b <= 255 and \
                        1 <= width <= 255 and off in [0, 1]):
                    print("  [오류] 입력값의 범위가 잘못되었습니다. 다시 입력해주세요.")
                    continue
                break
            except ValueError:
                print("  [오류] 숫자로 된 정수값만 입력해주세요.")
        
        # 서비스 요청 생성 및 전송
        request = SetPen.Request()
        request.r, request.g, request.b, request.width, request.off = r, g, b, width, off
        future = set_pen_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'Pen for "{turtle_name}" has been updated.')
        self.destroy_client(set_pen_client) # 동적 생성 클라이언트 파괴

    def handle_clear(self):
        """화면을 지우는 clear 서비스 요청을 보냅니다."""
        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Clear service not available, waiting...')
        
        future = self.clear_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Turtlesim screen has been cleared.')
        
    def run_manager(self):
        """노드의 메인 실행 로직"""
        # 시작 시 모든 거북이 제거
        time.sleep(1) # ROS Graph가 안정화될 시간 대기
        initial_turtles = self.get_turtle_list()
        if initial_turtles:
            self.get_logger().warn('--- Initializing: Killing all existing turtles ---')
            for turtle in initial_turtles:
                self.kill_turtle(turtle)
            self.get_logger().warn('--- Initialization complete ---')
        else:
            self.get_logger().info('No turtles found at startup.')

        # 사용자 명령 루프
        while rclpy.ok():
            input("\n명령을 시작하려면 Enter를 누르세요...")
            
            turtles = self.get_turtle_list()
            if not turtles:
                self.get_logger().warn("제어할 수 있는 거북이가 없습니다. 다른 터미널에서 거북이를 생성해주세요.")
                continue

            # 거북이 선택
            print("--- 제어할 거북이를 선택하세요 ---")
            for i, name in enumerate(turtles):
                print(f"{i + 1}: {name}")
            
            while True:
                try:
                    choice = int(input("번호 선택: ")) - 1
                    if 0 <= choice < len(turtles):
                        selected_turtle = turtles[choice]
                        break
                    else:
                        print("[오류] 목록에 있는 번호를 입력하세요.")
                except ValueError:
                    print("[오류] 숫자를 입력하세요.")
            
            # 명령어 선택
            print(f"\n--- '{selected_turtle}'에 대한 명령을 선택하세요 ---")
            print("1: SetPen (펜 설정)")
            print("2: Clear (화면 전체 지우기)")
            cmd = input("명령어 선택: ")

            if cmd == '1':
                self.handle_set_pen(selected_turtle)
            elif cmd == '2':
                self.handle_clear()
            else:
                print("[오류] 1 또는 2를 선택해주세요.")

def main(args=None):
    rclpy.init(args=args)
    manager_node = TurtleManagerClientNode()
    try:
        manager_node.run_manager()
    except KeyboardInterrupt:
        pass
    finally:
        manager_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
