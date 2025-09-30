import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import random
import math
import subprocess
import sys

# 사용할 메시지 및 서비스 타입
from turtlesim.srv import Spawn, Kill, SetPen, TeleportAbsolute
from std_srvs.srv import Empty


class TurtleManagerNode(Node):
    """
    사용자 입력을 받아 거북이를 관리하고, 각 거북이의 자율 주행을 제어하는 통합 노드
    """
    def __init__(self):
        super().__init__('turtle_manager_node')
        self.get_logger().info("Turtle Manager Node가 시작되었습니다.")

        # 자율 주행 노드 프로세스를 관리하는 딕셔너리
        self.drivers = {}  # { "turtle_name": subprocess.Popen_instance }

        # Turtlesim 서비스 클라이언트 생성
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')
        self.clear_client = self.create_client(Empty, 'clear')
        self.reset_client = self.create_client(Empty, 'reset')

    def user_spawn(self):
        """사용자 입력으로 새 거북이를 생성 (비동기)"""
        try:
            print("\n--- 새로운 거북이 생성 (spawn) ---")
            x = float(input("x 좌표를 입력하세요: "))
            y = float(input("y 좌표를 입력하세요: "))
            theta = float(input("바라볼 각도(theta, 라디안)를 입력하세요: "))
            name = input("새 거북이의 이름을 입력하세요 (예: turtle2): ")

            if not name:
                self.get_logger().warn("거북이 이름은 비워둘 수 없습니다. 작업을 취소합니다.")
                return

            spawn_req = Spawn.Request()
            spawn_req.x, spawn_req.y, spawn_req.theta, spawn_req.name = x, y, theta, name

            # 서비스를 비동기로 호출하고, 완료 시 처리할 콜백 함수를 등록합니다.
            future = self.spawn_client.call_async(spawn_req)
            future.add_done_callback(self.spawn_done_callback)
            
            self.get_logger().info(f"'{name}' 생성 요청을 보냈습니다. 결과는 곧 표시됩니다.")

        except ValueError:
            self.get_logger().error("잘못된 숫자 형식입니다. 작업을 취소합니다.")

    def spawn_done_callback(self, future):
        """spawn 서비스 호출 완료 시 실행되는 콜백"""
        try:
            result = future.result()
            new_name = result.name
            self.get_logger().info(f"성공: '{new_name}' 생성. 자율 주행 노드를 시작합니다.")
            if new_name not in self.drivers:
                # 새 거북이를 위한 드라이버 노드를 별도 프로세스로 실행
                driver_process = subprocess.Popen([sys.executable, 'turtle_driver_node.py', new_name])
                self.drivers[new_name] = driver_process
        except Exception as e:
            self.get_logger().error(f"spawn 서비스 호출 실패: {e}")

    def user_kill(self):
        """사용자 입력으로 특정 거북이를 제거"""
        name = input("\n제거할 거북이의 이름을 입력하세요: ")
        if not name:
            self.get_logger().warn("이름을 입력해야 합니다.")
            return

        # 1. 자율 주행 노드 프로세스 종료 및 딕셔너리에서 제거
        if name in self.drivers:
            self.get_logger().info(f"'{name}'의 Driver 노드를 종료합니다.")
            self.drivers[name].terminate() # SIGTERM 전송
            del self.drivers[name]
        else:
            self.get_logger().warn(f"'{name}'에 대한 Driver를 찾을 수 없습니다.")

        # 2. turtlesim에 거북이 제거 요청
        kill_req = Kill.Request(name=name)
        self.kill_client.call_async(kill_req)
        self.get_logger().info(f"'{name}' 제거 요청을 보냈습니다.")

    def user_set_pen(self):
        """사용자 입력으로 펜 설정을 변경"""
        try:
            name = input("\n펜 설정을 변경할 거북이 이름을 입력하세요 (기본값: turtle1): ") or "turtle1"
            if name not in self.get_turtle_list():
                self.get_logger().error(f"'{name}' 거북이를 찾을 수 없습니다.")
                return

            print(f"--- {name}의 펜 설정 변경 (set_pen) ---")
            r = int(input("빨강(r) 값을 입력하세요 (0-255): "))
            g = int(input("초록(g) 값을 입력하세요 (0-255): "))
            b = int(input("파랑(b) 값을 입력하세요 (0-255): "))
            width = int(input("펜의 두께(width)를 입력하세요: "))
            off_input = input("펜을 끄려면 '1' 또는 'y'를 입력하세요 (켜려면 Enter): ").lower()
            off = 1 if off_input in ['1', 'y', 'yes'] else 0

            pen_client = self.create_client(SetPen, f'/{name}/set_pen')
            if pen_client.wait_for_service(timeout_sec=1.0):
                pen_req = SetPen.Request(r=r, g=g, b=b, width=width, off=off)
                pen_client.call_async(pen_req)
                self.get_logger().info(f"'{name}'의 펜 설정 변경 요청을 보냈습니다.")
            else:
                self.get_logger().error(f"'{name}/set_pen' 서비스를 찾을 수 없습니다.")
            # 임시 클라이언트는 명시적으로 파괴하지 않아도 가비지 컬렉션됨

        except ValueError:
            self.get_logger().error("잘못된 숫자 형식입니다. 작업을 취소합니다.")

    def user_teleport_absolute(self):
        """사용자 입력으로 거북이를 절대 좌표로 이동"""
        try:
            name = input("\n순간이동할 거북이 이름을 입력하세요 (기본값: turtle1): ") or "turtle1"
            if name not in self.get_turtle_list():
                self.get_logger().error(f"'{name}' 거북이를 찾을 수 없습니다.")
                return

            print(f"--- {name} 절대 좌표로 이동 (teleport_absolute) ---")
            x = float(input("이동할 x 좌표를 입력하세요: "))
            y = float(input("이동할 y 좌표를 입력하세요: "))
            theta = float(input("바라볼 각도(theta, 라디안)를 입력하세요: "))

            teleport_client = self.create_client(TeleportAbsolute, f'/{name}/teleport_absolute')
            if teleport_client.wait_for_service(timeout_sec=1.0):
                teleport_req = TeleportAbsolute.Request(x=x, y=y, theta=theta)
                teleport_client.call_async(teleport_req)
                self.get_logger().info(f"'{name}' 이동 요청을 보냈습니다.")
            else:
                self.get_logger().error(f"'{name}/teleport_absolute' 서비스를 찾을 수 없습니다.")

        except ValueError:
            self.get_logger().error("잘못된 숫자 형식입니다. 작업을 취소합니다.")

    def user_clear(self):
        """배경을 초기화"""
        self.get_logger().info("배경 초기화 요청을 보냈습니다.")
        self.clear_client.call_async(Empty.Request())

    def user_reset(self):
        """Turtlesim을 리셋"""
        self.get_logger().info("Turtlesim 리셋 요청을 보냈습니다.")
        # 1. 모든 자율 주행 노드 프로세스 종료 및 딕셔너리 초기화
        for driver in list(self.drivers.values()):
            driver.terminate()
        self.drivers.clear()

        # 2. turtlesim 리셋 요청
        future = self.reset_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future) # 리셋이 완료될 때까지 대기

        # 3. 리셋 후 존재하는 모든 거북이를 찾아 자율 주행 시작
        self.get_logger().info("리셋 완료. 존재하는 거북이들을 다시 찾습니다...")
        import time
        time.sleep(1.0) # turtlesim이 리셋되고 토픽이 다시 생성될 시간을 줍니다.

        for turtle_name in self.get_turtle_list():
            if turtle_name and turtle_name not in self.drivers:
                self.get_logger().info(f"'{turtle_name}' 발견. 자율 주행 노드를 시작합니다.")
                driver_process = subprocess.Popen([sys.executable, 'turtle_driver_node.py', turtle_name])
                self.drivers[turtle_name] = driver_process

    def get_turtle_list(self):
        """현재 turtlesim에 존재하는 거북이 이름 목록을 반환"""
        turtle_names = []
        existing_topics = self.get_topic_names_and_types()
        for topic_name, _ in existing_topics:
            if topic_name.endswith('/pose'):
                turtle_name = topic_name.split('/')[1]
                if turtle_name:
                    turtle_names.append(turtle_name)
        return turtle_names


def print_menu():
    """사용자에게 보여줄 메뉴를 출력"""
    print("\n===== Turtle Manager 메뉴 =====")
    print("1: spawn (새 거북이 생성)")
    print("2: kill (특정 거북이 제거)")
    print("3: set_pen (펜 설정 변경)")
    print("4: teleport_absolute (절대 좌표로 이동)")
    print("5: clear (배경 초기화)")
    print("6: reset (Turtlesim 리셋)")
    print("q: 종료")
    print("==============================")

def main(args=None):
    rclpy.init(args=args)
    manager_node = TurtleManagerNode()

    # --- 수정된 부분: 현재 존재하는 모든 거북이를 찾아 자율 주행 시작 ---
    # 노드가 다른 노드의 토픽을 발견할 시간을 주기 위해 잠시 대기합니다.
    manager_node.get_logger().info("현재 존재하는 거북이들을 찾는 중...")
    import time
    time.sleep(1.0)

    # 프로그램 시작 시 존재하는 모든 거북이에 대해 드라이버 노드 실행
    for turtle_name in manager_node.get_turtle_list():
        if turtle_name not in manager_node.drivers:
            manager_node.get_logger().info(f"'{turtle_name}' 발견. 자율 주행 노드를 시작합니다.")
            driver_process = subprocess.Popen([sys.executable, 'turtle_driver_node.py', turtle_name])
            manager_node.drivers[turtle_name] = driver_process

    # 여러 스레드를 사용하여 콜백들이 서로를 막지 않도록 함
    # 자율주행(타이머)과 사용자 입력(input)이 동시에 동작해야 하므로 스레드 분리가 중요
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(manager_node)

    # 별도 스레드에서 rclpy.spin을 실행하여 ROS 콜백 처리
    import threading
    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()

    # 메인 스레드에서는 사용자 입력을 처리
    while rclpy.ok():
        print_menu()
        choice = input("실행할 명령 번호를 선택하세요: ")
        if choice == '1': manager_node.user_spawn()
        elif choice == '2': manager_node.user_kill()
        elif choice == '3': manager_node.user_set_pen()
        elif choice == '4': manager_node.user_teleport_absolute()
        elif choice == '5': manager_node.user_clear()
        elif choice == '6': manager_node.user_reset()
        elif choice.lower() == 'q':
            break
        else:
            print("잘못된 입력입니다. 메뉴에 있는 번호를 입력해주세요.")

    # 프로그램 종료 시 모든 자식 프로세스(드라이버 노드) 종료
    for driver_process in manager_node.drivers.values():
        driver_process.terminate()

    rclpy.shutdown()
    spin_thread.join()
    print("프로그램을 종료합니다.")

if __name__ == '__main__':
    main()
