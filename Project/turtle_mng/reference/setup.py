from setuptools import find_packages, setup

package_name = "my_first_pkg"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # ament 인덱스에 패키지를 등록한다 — 이 항목이 없으면 `ros2 pkg list` 에 나타나지 않는다.
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Daesung Kim",
    maintainer_email="daesung7723@gmail.com",
    description="turtlesim 을 제어하는 명령행 도구 모음 — ROS2·Python 학습용 독립 미니프로젝트",
    license="MIT",
    # `ros2 run my_first_pkg <이름>` 으로 실행되는 프로그램 목록.
    # 왼쪽 = 실행 이름, 오른쪽 = 모듈:함수. 도구 하나 = 파일 하나 = main() 하나.
    entry_points={
        "console_scripts": [
            "list = my_first_pkg.list_turtles:main",
            "spawn = my_first_pkg.spawn:main",
            "kill = my_first_pkg.kill:main",
            "random_move = my_first_pkg.random_move:main",
            "set_pen = my_first_pkg.set_pen:main",
            "stop = my_first_pkg.stop:main",
        ],
    },
)
