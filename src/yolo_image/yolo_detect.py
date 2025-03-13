import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
import numpy as np
import json
from SparkYolo import sparkYolo


# Author: Karl.Kwon
# Email: mrthinks@gmail.com

mtx = np.array([[1288.64415,    0.     ,  636.22318],
        [0.     , 1294.26622,  450.07389],
        [0.     ,    0.     ,    1.     ]])
dist = np.array([[0.051971, 0.053343, 0.024273, 0.000262, 0.000000]])

model_path = 'yolov8s_trained.pt'
# pix_2_mm = 0.00012 # distance(m) - 175mm
pix_2_mm = 0.000153 # distance(m) - mm

class YoloDetect(Node):
    def __init__(self):
        super().__init__('yolo_detect')
        self.yolo_infer = sparkYolo(model_path)

        self.subscription_rgb = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.listener_callback_rgb, 1)
        self.subscription_rgb  # prevent unused variable warning

        self.img_publisher_ = self.create_publisher(CompressedImage, 'yolo/compressed', 1)
        self.info_publisher_ = self.create_publisher(String, 'yolo/detected_info', 5)

    def publish_img(self, frame):
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]  # 90은 압축 품질
        _, compressed_image = cv2.imencode('.jpg', frame, encode_param)

        # 압축된 이미지를 CompressedImage 메시지로 변환
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()  # 타임스탬프 추가
        msg.header.frame_id = "camera"  # 프레임 ID 설정
        msg.format = "jpeg"  # 압축 형식 설정
        msg.data = compressed_image.tobytes()  # 압축된 이미지 데이터

        # CompressedImage 퍼블리시
        self.img_publisher_.publish(msg)
        # self.get_logger().info('Publishing compressed image...')

    def publish_info(self, str_):
        msg = String()
        msg.data = str_
        self.info_publisher_.publish(msg)

    def listener_callback_rgb(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode to color image

        h,  w = image_np.shape[:2]
        ####################################################################3    
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

        # undistort
        image_undistort = cv2.undistort(image_np, mtx, dist, None, newcameramtx)

        plots, info = self.yolo_infer.inference(image_undistort, pix_2_mm)
        info = json.dumps(info) 
        # print(info)

        self.publish_info(info)
        self.publish_img(plots)


# ros2 topic echo /yolo/detected_info

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = YoloDetect()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
