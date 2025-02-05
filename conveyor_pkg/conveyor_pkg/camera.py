import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # 이미지 퍼블리셔 생성
        self.publisher_ = self.create_publisher(CompressedImage, 'image_raw/compressed', 10)
        
        # 카메라 설정
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        # self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('카메라를 열 수 없습니다.')
            return
            
        # 카메라 설정 순서 변경
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FPS, 25)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        # 설정 확인
        actual_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f'Camera resolution: {actual_w}x{actual_h}, FPS: {actual_fps}')

        # 타이머 설정 (25fps에 맞춤)
        self.timer = self.create_timer(1.0/25.0, self.publish_image)

    def publish_image(self):
        if not self.cap.isOpened():
            self.get_logger().error('카메라가 연결되어 있지 않습니다.')
            return
            
        ret, frame = self.cap.read()
        if ret:
            try:
                # JPEG 압축
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                _, compressed_image = cv2.imencode('.jpg', frame, encode_param)

                # CompressedImage 메시지 생성
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "camera"
                msg.format = "jpeg"
                msg.data = compressed_image.tobytes()

                # 발행
                self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().error(f'이미지 처리 중 오류 발생: {str(e)}')
        else:
            self.get_logger().warn('프레임을 읽을 수 없습니다.')

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(camera_node, 'cap') and camera_node.cap.isOpened():
            camera_node.cap.release()
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
