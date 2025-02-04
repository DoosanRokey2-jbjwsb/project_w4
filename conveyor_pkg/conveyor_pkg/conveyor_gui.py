import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import json
import threading
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class ConveyorGUI(Node):
    def __init__(self, window):
        super().__init__('conveyor_gui')
        self.window = window
        
        # Status 구독
        self.status_subscription = self.create_subscription(
            String,
            'conveyor/status',
            self.status_callback,
            10)
            
        # Control 발행자
        self.control_publisher = self.create_publisher(
            String,
            'conveyor/control',
            10)
            
        # 카메라 이미지 구독
        self.camera_subscription = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.image_callback,
            10)
    
    def status_callback(self, msg):
        try:
            status_data = json.loads(msg.data)
            self.window.update_status_signal.emit(status_data['status'])
        except Exception as e:
            self.get_logger().error(f'Status update error: {e}')
    
    def send_control_command(self, command, distance=None):
        try:
            msg = String()
            control_data = {
                "control": command
            }
            if distance:
                control_data["distance.mm"] = distance
            msg.data = json.dumps(control_data)
            self.control_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Control command error: {e}')

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if image is None:
                self.get_logger().warn('Failed to decode image')
                return
                
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            height, width, channel = image.shape
            bytes_per_line = 3 * width
            q_image = QImage(image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            self.window.update_image_signal.emit(q_image)
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')

class WindowClass(QMainWindow):
    update_status_signal = pyqtSignal(str)
    update_image_signal = pyqtSignal(QImage)
    
    def __init__(self):
        super().__init__()
        self.initUI()
        self.initROS()
        
        # 시그널 연결
        self.update_status_signal.connect(self.update_status_label)
        self.update_image_signal.connect(self.update_image)
    
    def initUI(self):
        # 윈도우 기본 설정
        self.setGeometry(100, 100, 800, 600)
        self.setWindowTitle('Conveyor Control GUI')
        
        # 이미지 표시 영역 (중앙)
        self.image_label = QLabel(self)
        self.image_label.setGeometry(50, 50, 500, 400)
        self.image_label.setStyleSheet("""
            QLabel {
                border: 1px solid black;
                background-color: white;
            }
        """)
        self.image_label.setAlignment(Qt.AlignCenter)
        
        # 상태 정보창 (우측 상단)
        self.status_label = QTextEdit(self)
        self.status_label.setGeometry(600, 50, 150, 200)        # 마지막이 높이값(세로 위아래 길이)
        self.status_label.setReadOnly(True)
        self.status_label.setStyleSheet("""
            QTextEdit {
                border: 1px solid black;
                background-color: white;
                font-size: 12px;
            }
        """)
        
        # 이동 버튼 (좌측 하단)
        self.move_btn = QPushButton("이동", self)
        self.move_btn.setGeometry(50, 500, 150, 50)
        self.move_btn.clicked.connect(self.move_button_clicked)
        
        # 정지 버튼 (중앙 하단)
        self.stop_btn = QPushButton("정지", self)
        self.stop_btn.setGeometry(250, 500, 150, 50)
        self.stop_btn.clicked.connect(self.stop_button_clicked)
        
        # 거리 입력 필드
        self.distance_input = QLineEdit(self)
        self.distance_input.setGeometry(50, 450, 150, 40)
        self.distance_input.setPlaceholderText("거리(mm)")
    
    def initROS(self):
        self.ros_node = ConveyorGUI(self)
        self.ros_thread = threading.Thread(target=self.ros_spin)
        self.ros_thread.daemon = True
        self.ros_thread.start()
    
    def ros_spin(self):
        while rclpy.ok():
            rclpy.spin_once(self.ros_node, timeout_sec=0.1)
    
    @pyqtSlot(str)
    def update_status_label(self, status):
        if status != getattr(self, 'previous_status', None):
            current_time = QDateTime.currentDateTime().toString("hh:mm:ss")
            log_text = f"[{current_time}] Status: {status}\n"
            self.status_label.append(log_text)
            self.previous_status = status
            
            # 스크롤을 최하단으로
            self.status_label.verticalScrollBar().setValue(
                self.status_label.verticalScrollBar().maximum()
            )
    
    @pyqtSlot(QImage)
    def update_image(self, q_image):
        try:
            scaled_image = q_image.scaled(self.image_label.width(), 
                                        self.image_label.height(), 
                                        Qt.KeepAspectRatio)
            self.image_label.setPixmap(QPixmap.fromImage(scaled_image))
        except Exception as e:
            print(f"Image update error: {e}")
    
    def move_button_clicked(self):
        try:
            distance = int(self.distance_input.text())
            self.ros_node.send_control_command('go', distance)
        except ValueError:
            QMessageBox.warning(self, "입력 오류", "올바른 거리값을 입력하세요.")
    
    def stop_button_clicked(self):
        self.ros_node.send_control_command('stop')
    
    def closeEvent(self, event):
        self.ros_node.destroy_node()
        event.accept()

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    window = WindowClass()
    window.show()
    
    try:
        app.exec_()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
