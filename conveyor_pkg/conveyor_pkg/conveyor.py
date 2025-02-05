import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import json
import time
import threading

class ConveyorNode(Node):
    def __init__(self):
        super().__init__('conveyor_node')
        
        # Publisher & Subscription
        self.status_publisher = self.create_publisher(String, 'conveyor/status', 10)
        self.control_subscription = self.create_subscription(
            String,
            'conveyor/control',
            self.control_callback,
            10)
            
        # Serial
        self.ser = None
        self.connect_serial()
        
        # Serial Thread
        self.serial_thread = threading.Thread(target=self.serial_loop)
        self.serial_thread.daemon = True
        self.serial_thread.start()
    
    def connect_serial(self):
        try:
            # 이전 연결 정리
            if hasattr(self, 'ser') and self.ser:
                self.ser.close()
                time.sleep(1)
                
            self.ser = serial.Serial(
                port='/dev/ttyACM1',
                baudrate=115200,
                timeout=1
            )
            
            time.sleep(2)  # 아두이노 리셋 대기
            
            if self.ser.is_open:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                self.connected = True
                self.get_logger().info("Serial connected")
                
                # (재 연결후?) READY 상태 발행
                status_data = {
                    "status": "READY",
                    "timestamp": time.time()
                }
                msg = String()
                msg.data = json.dumps(status_data)
                self.status_publisher.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            self.connected = False
    
    def control_callback(self, msg):
        try:
            control_data = json.loads(msg.data)
            command = control_data.get('control')
            
            if command == 'go':
                distance_mm = control_data.get('distance.mm')
                steps = int(distance_mm * 10.4)  # 1mm당 10.4스텝
                self.send_command_to_arduino(str(steps))
            elif command == 'stop':
                self.send_command_to_arduino('0')
                
        except Exception as e:
            self.get_logger().error(f'Control error: {e}')
    
    def send_command_to_arduino(self, command: str):
        try:
            if self.ser and self.ser.is_open:
                self.ser.write((command + '\n').encode('utf-8'))
            else:
                self.get_logger().error("Serial not connected")
        except Exception as e:
            self.get_logger().error(f'Send command error: {e}')

    def serial_loop(self):
        previous_status = None
        while True:
            try:
                if not self.ser or not self.ser.is_open:
                    if previous_status != "DISCONNECTED":
                        status_data = {
                            "status": "DISCONNECTED",
                            "timestamp": time.time()
                        }
                        msg = String()
                        msg.data = json.dumps(status_data)
                        self.status_publisher.publish(msg)
                        previous_status = "DISCONNECTED"
                    time.sleep(1)
                    try:
                        self.connect_serial()
                    except:
                        continue
                    continue

                if self.ser.in_waiting:
                    data = self.ser.read().decode()
                    current_status = "RUN" if data == '_' else "READY" if data == '.' else data
                    
                    if current_status != previous_status:
                        status_data = {
                            "status": current_status,
                            "timestamp": time.time()
                        }
                        msg = String()
                        msg.data = json.dumps(status_data)
                        self.status_publisher.publish(msg)
                        previous_status = current_status

            except (serial.SerialException, OSError) as e:  # OSError 추가
                if previous_status != "DISCONNECTED":
                    status_data = {
                        "status": "DISCONNECTED",
                        "timestamp": time.time()
                    }
                    msg = String()
                    msg.data = json.dumps(status_data)
                    self.status_publisher.publish(msg)
                    previous_status = "DISCONNECTED"
                
                self.ser = None
                time.sleep(1)



def main(args=None):
    rclpy.init(args=args)
    node = ConveyorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
