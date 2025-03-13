# dataset/train.py 파일 생성
from ultralytics import YOLO

# 모델 초기화
model = YOLO('yolov8n.pt')

# 학습 시작
model.train(
    data='data.yaml',
    epochs=20,
    imgsz=1280,
    batch=4,
    name='box_detection'
)

