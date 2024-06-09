from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from msgs_clase.msg import Dotline   # type: ignore

from yolov8_msgs.msg import InferenceResult # type: ignore
from yolov8_msgs.msg import Yolov8Inference # type: ignore

bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.model = YOLO('/home/abraham/modelos/dotVertical.pt')

        self.yolov8_inference = Yolov8Inference()

        self.subscription = self.create_subscription(
            Image,
            '/sign_information',
            self.camera_callback,
            10) 

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference_vertical", 1)
        self.predi_pub = self.create_publisher(Dotline, "/vertical_bool", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result_vertical", 1)
        
        self.timer_period = 0.2
        self.timer = self.create_timer(self.timer_period, self.timer_callback_signs)

        self.img = np.ones((480, 640, 3), dtype=np.uint8)

        #Centro de la imágen
        self.img_center = 340/2  # Width of the image
    
        
        self.get_logger().info('Sign detection node initialized')

    def camera_callback(self, data):
        if data is not None:
            self.img = bridge.imgmsg_to_cv2(data, "bgr8")

    def timer_callback_signs(self):
        results = self.model(source=self.img, conf=0.4, verbose=False)
        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inference_result = InferenceResult()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls
                self.inference_result.class_name = self.model.names[int(c)]
                self.inference_result.top = int(b[1])
                self.inference_result.left = int(b[0])
                self.inference_result.bottom = int(b[3])
                self.inference_result.right = int(b[2])
                self.yolov8_inference.yolov8_inference.append(self.inference_result)

        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")

        self.img_pub.publish(img_msg)
        #self.yolov8_pub.publish(self.yolov8_inference)

        self.escribirMensaje(self.yolov8_inference)

        self.yolov8_inference.yolov8_inference.clear()

    def escribirMensaje(self, yoloInference):
        self.senialesDetectadas = Dotline()

        for inference in yoloInference.yolov8_inference:
            class_name = inference.class_name
            
            center_x = (inference.left + inference.right) / 2
            
            if class_name == "Vertical-dotline":
                if center_x >  self.img_center: 
                    self.senialesDetectadas.right = True
                else: 
                    self.senialesDetectadas.left = True

        if self.senialesDetectadas.left and self.senialesDetectadas.right:
            self.senialesDetectadas.left = False
            self.senialesDetectadas.right = False
            self.senialesDetectadas.both = True
    
        self.predi_pub.publish(self.senialesDetectadas)


def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
