import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from turtlesim.srv import SetPen

class TurtleServiceClientNode(Node):
    def __init__(self):
        super().__init__('turtle_service_client')
        
        # 1. 각 서비스에 대한 클라이언트 생성
        self.reset_client = self.create_client(Empty, '/reset')
        self.clear_client = self.create_client(Empty, '/clear')
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        # 서비스 서버가 준비될 때까지 대기
        self.wait_for_services()

    def wait_for_services(self):
        """turtlesim 서비스 서버가 실행될 때까지 기다립니다."""
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Reset service not available, waiting again...')
        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Clear service not available, waiting again...')
        while not self.set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetPen service not available, waiting again...')
        self.get_logger().info('All services are now available.')

    def send_reset_request(self):
        """reset 서비스에 요청을 보냅니다."""
        request = Empty.Request()
        future = self.reset_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Turtlesim has been reset.')
        else:
            self.get_logger().error('Exception while calling service: %r' % future.exception())

    def send_clear_request(self):
        """clear 서비스에 요청을 보냅니다."""
        request = Empty.Request()
        future = self.clear_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Turtlesim screen has been cleared.')
        else:
            self.get_logger().error('Exception while calling service: %r' % future.exception())

    def send_set_pen_request(self, r, g, b, width, off):
        """set_pen 서비스에 요청을 보냅니다."""
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off
        
        future = self.set_pen_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Pen settings have been updated.')
        else:
            self.get_logger().error('Exception while calling service: %r' % future.exception())

def main(args=None):
    rclpy.init(args=args)
    service_client_node = TurtleServiceClientNode()

    # 사용자 입력을 받기 위한 메인 루프
    while rclpy.ok():
        print("\n--- Turtlesim 제어 명령어 ---")
        print("1: reset (거북이 위치 초기화)")
        print("2: clear (그림판 초기화)")
        print("3: set_pen (펜 설정 변경)")
        print("Ctrl+C 로 종료")
        
        command = input("명령어를 선택하세요: ")

        if command == '1':
            service_client_node.send_reset_request()
        elif command == '2':
            service_client_node.send_clear_request()
        elif command == '3':
            while True: # 올바른 값이 입력될 때까지 반복
                try:
                    r = int(input("  - R (0-255): "))
                    g = int(input("  - G (0-255): "))
                    b = int(input("  - B (0-255): "))
                    if not (0 <= r <= 255 and 0 <= g <= 255 and 0 <= b <= 255):
                        print("  [오류] R, G, B 값은 0과 255 사이여야 합니다. 다시 입력해주세요.")
                        continue

                    width = int(input("  - 두께 (1-255): "))
                    if not (1 <= width <= 255):
                         print("  [오류] 두께 값은 1과 255 사이여야 합니다. 다시 입력해주세요.")
                         continue
                         
                    off = int(input("  - 선 표시 끄기 (0: 켜기, 1: 끄기): "))
                    if off not in [0, 1]:
                        print("  [오류] 0 또는 1만 입력할 수 있습니다. 다시 입력해주세요.")
                        continue
                        
                    # 모든 입력이 정상이면 루프 탈출
                    break
                except ValueError:
                    print("  [오류] 숫자로 된 정수값만 입력해주세요.")
            
            service_client_node.send_set_pen_request(r, g, b, width, off)
        else:
            print("[오류] 1, 2, 3 중 하나를 선택해주세요.")

    service_client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
