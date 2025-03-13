import rclpy
from rclpy.node import Node
from aruco_msgs.msg import MarkerArray, Marker
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, PoseArray
from turtlebot_cosmo_interface.srv import MoveitControl
from srv_call_test import TurtlebotArmClient
import time
import ast

class IntegratedProcess(Node):
    def __init__(self):
        super().__init__('integrated_process')

        # Aruco Marker Listener 설정
        self.aruco_sub = self.create_subscription(
            MarkerArray,
            'detected_markers',
            self.aruco_listener_callback,
            10)
        
        # Yolo Detection Listener 설정
        self.yolo_sub = self.create_subscription(
            String,
            '/yolo/detected_info',
            self.yolo_listener_callback,
            10)
        
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 2)
        
        # 상태 변수
        self.aruco_marker_found = False
        self.task_completed = False
        self.yolofind = False
        self.armrun = False
        self.yolo_x = 0
        self.yolo_y = 0
        self.marker_id = None
        self.state = 'START'  

        self.count = 0

        self.aruco_pose = None

        self.create_timer(1.0, self.run_tasks)

    # Aruco 리스너 콜백 함수는 변경 없음

    # YOLO 리스너 콜백 함수
    def yolo_listener_callback(self, msg):
        if self.state not in ('YOLO', 'PURPLE'):
            return

        if not self.armrun:
            data = msg.data
            try:
                data_list = ast.literal_eval(data)
                if len(data_list) > 0:
                    self.yolo_x = data_list[0][1]
                    self.yolo_y = data_list[0][2]

                    print(f"Detected coordinates: {self.yolo_x}, {self.yolo_y}")
                    print("done")

                    if self.state == 'YOLO':
                        if not self.yolofind:
                            self.yolofind = True
                            self.yolo_arm_controll()
                    elif self.state == 'PURPLE':
                        if not self.yolofind:
                            self.yolofind = True
                            self.purple_arm_control()
            except Exception as e:
                self.get_logger().error(f"Error processing the data: {e}")


    # YOLO 로봇 팔 컨트롤 함수: detect 이후에 group 이름들을 나열해서 순차 동작하게 만드는 함수
    def yolo_arm_controll(self):
        arm_client = TurtlebotArmClient()
        # 서비스 요청을 해서 moveit에게 움직이라고 요청함
        print("task start!")
        print(f"Get coordinates: {self.yolo_x}, {self.yolo_y}")

        if self.yolofind:
            self.armrun = True

            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

            pose_array = self.append_pose_init(0.137496 - self.yolo_y + 0.07, 0.0 - self.yolo_x + 0.02, 0.122354)
            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

            pose_array = self.append_pose_init(0.137496 - self.yolo_y + 0.07, 0.0 - self.yolo_x + 0.02, 0.082354)
            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')

            response = arm_client.send_request(2, "close")
            arm_client.get_logger().info(f'Response: {response.response}')

            response = arm_client.send_request(1, "home2")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

            print("conveyor task start")

            response = arm_client.send_request(1, "conveyor_up")
            arm_client.get_logger().info(f'Response: {response.response}')

            response = arm_client.send_request(1, "test_conveyor")
            arm_client.get_logger().info(f'Response: {response.response}')

            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')

            print("throw")

            response = arm_client.send_request(1, "conveyor_up")
            arm_client.get_logger().info(f'Response: {response.response}')

            response = arm_client.send_request(1, "camera_home")
            arm_client.get_logger().info(f'Response: {response.response}')

            time.sleep(3)

            print("jobs_done")

            self.armrun = False
            self.yolofind = False
            self.count += 1

            if self.count == 1:
                self.home2_arm_controll()
                self.state = 'BACKWARD'

    # 나머지 함수들은 변경 없음

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedProcess()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
