import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class Fase3ColorDetector(Node):
    def __init__(self):
        super().__init__('fase3_color_detector')
        
        # Declare parameters
        self.declare_parameter('image_topic', '/vertical_camera')
        self.declare_parameter('area_minima_bases', 300)
        
        # Declare HSV parameters for each shape
        self.shape_names = ['circulo', 'triangulo', 'hexagono', 'hexagono2', 'pentagono', 
                           'estrela', 'cruz', 'casa', 'quadrado', 'quadrado2']
        
        for shape in self.shape_names:
            self.declare_parameter(f'{shape}.hue_lower', 0)
            self.declare_parameter(f'{shape}.hue_upper', 180)
            self.declare_parameter(f'{shape}.saturation_lower', 0)
            self.declare_parameter(f'{shape}.saturation_upper', 255)
            self.declare_parameter(f'{shape}.value_lower', 0)
            self.declare_parameter(f'{shape}.value_upper', 255)
        
        # Get parameter values
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        
        # Load HSV ranges from ROS2 parameters
        self.hsv_ranges = {}
        for shape in self.shape_names:
            self.hsv_ranges[shape] = {
                'hue_lower': self.get_parameter(f'{shape}.hue_lower').get_parameter_value().integer_value,
                'hue_upper': self.get_parameter(f'{shape}.hue_upper').get_parameter_value().integer_value,
                'saturation_lower': self.get_parameter(f'{shape}.saturation_lower').get_parameter_value().integer_value,
                'saturation_upper': self.get_parameter(f'{shape}.saturation_upper').get_parameter_value().integer_value,
                'value_lower': self.get_parameter(f'{shape}.value_lower').get_parameter_value().integer_value,
                'value_upper': self.get_parameter(f'{shape}.value_upper').get_parameter_value().integer_value,
            }
        
        self.get_logger().info(f'Loaded HSV ranges for shapes: {list(self.hsv_ranges.keys())}')
        
        self.bridge = CvBridge()
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriber = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile
        )
        self.image_pub = self.create_publisher(Image, '/fase3_color_detector/image', 10)
        
        # Para envio das info de deteccao para o ros2
        self.classification_topic = "/vertical_camera/classification"
        self.classification_publisher = self.create_publisher(Detection2DArray, self.classification_topic, 10)


    def image_callback(self, msg):
        detection_array = Detection2DArray()
        detection_array.header = msg.header
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {str(e)}")
            return
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        annotated = frame.copy()
        height, width = frame.shape[:2]  # Para normalização
        for color, params in self.hsv_ranges.items():
            lower = np.array([
                params['hue_lower'],
                params['saturation_lower'],
                params['value_lower']
            ])
            upper = np.array([
                params['hue_upper'],
                params['saturation_upper'],
                params['value_upper']
            ])
            mask = cv2.inRange(hsv, lower, upper)
            # Morphology to clean up
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > self.get_parameter('area_minima_bases').get_parameter_value().integer_value:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(annotated, (x, y), (x+w, y+h), (0,255,0), 2)
                    cv2.putText(annotated, color, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

                    # Preencher Detection2D para cada base detectada (normalizado)
                    det = Detection2D()
                    det.header = msg.header
                    det.bbox = BoundingBox2D()
                    det.bbox.center.position.x = (x + w/2) / width
                    det.bbox.center.position.y = (y + h/2) / height
                    det.bbox.size_x = w / width
                    det.bbox.size_y = h / height
                    hypo = ObjectHypothesisWithPose()
                    hypo.hypothesis.class_id = color
                    hypo.hypothesis.score = 1.0  # Score fixo, pois é segmentação clássica
                    det.results.append(hypo)
                    detection_array.detections.append(det)
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            self.image_pub.publish(annotated_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert annotated image: {str(e)}")
        # Publicar detections
        self.classification_publisher.publish(detection_array)


def main(args=None):
    rclpy.init(args=args)
    node = Fase3ColorDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
