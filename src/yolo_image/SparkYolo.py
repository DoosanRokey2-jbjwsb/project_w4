from ultralytics import YOLO
import cv2

class sparkYolo:
    def __init__(self, model_path='yolov8s_trained.pt'):
        print('model loading...')
        self.model = YOLO(model_path)
        print('model load done.')

    def convert_box_center_2_mm(self, results, img_size, pix_2_mm):
        cls = results[0].boxes.cls.tolist()
        centers = []
        
        for i, r in enumerate(results[0].boxes.xywh):
            d = r.tolist()
            centers.append( (int(cls[i]), (d[0] - img_size[0]/2)*pix_2_mm, (d[1] - img_size[1]/2)*pix_2_mm) )
        
        return centers

    def inference(self, img_color, pix_2_mm):
        results = self.model(img_color)
        plots = results[0].plot()
        shp = img_color.shape

        plots = cv2.line(plots, (0, int(shp[0]/2)), (shp[1], int(shp[0]/2)), (0, 255, 255), 3)
        plots = cv2.line(plots, (int(shp[1]/2), 0), (int(shp[1]/2), shp[0]-1), (0, 255, 255), 3)
        ret = self.convert_box_center_2_mm(results, (shp[1], shp[0]), pix_2_mm)

        return plots, ret


if __name__ == '__main__':
    infer = sparkYolo()
    pix_2_mm = 0.12

    # img_path = './distance_check.jpg'
    img_path = "./my_data/valid/images/saved_image_035.jpg"
    img_color = cv2.imread(img_path)

    plots, info = infer.inference(img_color, pix_2_mm)
    print(info)

    # plt.imshow(cv2.cvtColor(plots, cv2.COLOR_BGR2RGB))
    # plt.show()
    cv2.imshow('', plots)
    cv2.waitKey(0)
