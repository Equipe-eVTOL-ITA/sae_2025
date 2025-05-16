import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from custom_msgs.msg import BarCode, MultiBarCode
from cv_bridge import CvBridge
import cv2
import numpy as np
import imutils

class BarcodeSubscriber(Node):
    def __init__(self):
        super().__init__('barcode_subscriber')
        
        # Subscriber for the compressed image
        self.subscription = self.create_subscription(
            CompressedImage,
            'camera/image/compressed',
            self.image_callback,
            10)
        
        # Publisher for the annotated image
        self.image_publisher = self.create_publisher(CompressedImage, 'camera/annotated/image/compressed', 10)
        
        # Publisher for each bounding box info
        self.bbox_publisher = self.create_publisher(BarCode, 'barcode/bounding_box', 10)
        
        self.bridge = CvBridge()
        self.scale_factor = 2.0  # Scale factor to amplify the image

    def image_callback(self, msg):
        # Convert ROS compressed image message to OpenCV format
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        # Amplify the image by resizing
        amplified_frame = cv2.resize(frame, None, fx=self.scale_factor, fy=self.scale_factor, interpolation=cv2.INTER_LINEAR)
        
        # Detect barcodes in the amplified frame
        annotated_frame, bbox_msgs = self.detect_barcode(amplified_frame, frame.shape)
        
        # Publish bounding box info for each detected barcode
        for bbox_msg in bbox_msgs:
            self.bbox_publisher.publish(bbox_msg)
        
        # Resize back to original size before publishing
        final_frame = cv2.resize(annotated_frame, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_LINEAR)
        
        # Convert annotated frame to a compressed ROS image message
        annotated_msg = self.bridge.cv2_to_compressed_imgmsg(final_frame)
        
        # Publish the annotated image
        self.image_publisher.publish(annotated_msg)

    def detect_barcode(self, image, original_shape):
        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Compute the gradients in x and y directions
        ddepth = cv2.CV_32F
        gradX = cv2.Sobel(gray, ddepth=ddepth, dx=1, dy=0, ksize=-1)
        gradY = cv2.Sobel(gray, ddepth=ddepth, dx=0, dy=1, ksize=-1)

        # Subtract the y-gradient from the x-gradient to reveal the barcode region
        gradient = cv2.subtract(gradX, gradY)
        gradient = cv2.convertScaleAbs(gradient)

        # Blur and threshold the gradient image
        blurred = cv2.blur(gradient, (9, 9))
        _, thresh = cv2.threshold(blurred, 225, 255, cv2.THRESH_BINARY)

        # Apply morphological operations
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 7))
        closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        closed = cv2.erode(closed, None, iterations=4)
        closed = cv2.dilate(closed, None, iterations=4)

        # Find contours in the thresholded image
        cnts = cv2.findContours(closed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        bbox_msgs = []
        for c in cnts:
            # Compute the rotated bounding box of each contour
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect) if imutils.is_cv4() else cv2.cv.BoxPoints(rect)
            box = np.int0(box)
            
            # Draw a bounding box around the detected barcode
            cv2.drawContours(image, [box], -1, (0, 255, 0), 3)

            # Calculate bounding box center and height as normalized values
            box_x, box_y, box_w, box_h = cv2.boundingRect(c)
            center_x = (box_x + box_w / 2) / original_shape[1]
            center_y = (box_y + box_h / 2) / original_shape[0]
            height = box_h / original_shape[0]

            # Create BarCode message with normalized values
            bbox_msg = BarCode()
            bbox_msg.center_x = float(center_x)
            bbox_msg.center_y = float(center_y)
            bbox_msg.height = float(height)

            # Add the bounding box message to the list
            bbox_msgs.append(bbox_msg)

        return image, bbox_msgs

def main(args=None):
    rclpy.init(args=args)
    barcode_subscriber = BarcodeSubscriber()
    rclpy.spin(barcode_subscriber)
    barcode_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
