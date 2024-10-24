# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, LaserScan
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point
# from ultralytics import YOLO
# import cv2
# import numpy as np
# from cv_bridge import CvBridge

# from std_msgs.msg import Float32MultiArray

# class PersonDetector(Node):
#     def __init__(self):
#         super().__init__('person_detector')
        
#         # YOLOv8 model from Ultralytics
#         self.model = YOLO('yolov8n.pt')  # Use 'yolov8n.pt' or any other pretrained model
#         self.bridge = CvBridge()

#         # Publishers and subscribers
#         self.camera_subscriber = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
#         self.laser_subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
#         self.overlay_publisher = self.create_publisher(Image, '/camera/image_overlay', 10)
#         self.angle_distance_publisher = self.create_publisher(Float32MultiArray, '/person_angle_distance', 10)

#         # Variables for storing data
#         self.latest_laser_scan = None
#         self.people_positions = []

#         # Camera parameters (not sure on specs will have to investigate)
#         self.camera_fov = 60.0  # Horizontal field of view in degrees
#         self.image_width = 640  # Width of the image in pixels

#     def image_callback(self, msg):
#         # Convert image from ROS to OpenCV
#         image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

#         # Run YOLO detection
#         results = self.model(image)

#         person_angle = None

#         # Loop through the results and draw bounding boxes
#         for result in results[0].boxes:
#             # Extract the bounding box coordinates
#             x1, y1, x2, y2 = map(int, result.xyxy[0])

#             # Extract the object label and confidence
#             confidence = result.conf[0]
#             class_id = int(result.cls[0])
#             label = self.model.names[class_id]  # Get the class label

#             # Draw the bounding box
#             cv2.rectangle(image, (x1, y1), (x2, y2), (64, 224, 208), 2)

#             # Create the label text with confidence
#             label_text = f"{label}: {confidence:.2f}"

#             # Put the label text on the frame
#             cv2.putText(image, label_text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (64, 224, 208), 2)

#         # Publish the overlay image with bounding boxes
#         overlay_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
#         self.overlay_publisher.publish(overlay_msg)

        

# def main(args=None):
#     rclpy.init(args=args)
#     node = PersonDetector()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from ultralytics import YOLO
import cv2
import numpy as np
from cv_bridge import CvBridge

class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')
        
        # YOLOv8 model from Ultralytics
        self.model = YOLO('yolov8n.pt')  # Use 'yolov8n.pt' or any other pretrained model
        self.bridge = CvBridge()

        # Publishers and subscribers
        self.camera_subscriber = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.laser_subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.overlay_publisher = self.create_publisher(Image, '/camera/image_overlay', 10)
        self.angle_distance_publisher = self.create_publisher(Float32MultiArray, '/person_angle_distance', 10)

        # Variables for storing data
        self.latest_laser_scan = None
        self.people_positions = []

        # Camera parameters (to be adjusted based on your camera specifications)
        self.camera_fov = 90.0  # Horizontal field of view in degrees
        self.image_width = 640  # Width of the image in pixels

    def image_callback(self, msg):
        # Convert image from ROS to OpenCV
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Run YOLO detection
        results = self.model(image)

        person_angle = None

        # Loop through the results and draw bounding boxes
        for result in results[0].boxes:
            # Extract the bounding box coordinates
            x1, y1, x2, y2 = map(int, result.xyxy[0])

            # Extract the object label and confidence
            confidence = result.conf[0]
            class_id = int(result.cls[0])
            label = self.model.names[class_id]  # Get the class label

            if label == 'person':  # Only process person detections
                # Draw the bounding box
                cv2.rectangle(image, (x1, y1), (x2, y2), (64, 224, 208), 2)

                # Create the label text with confidence
                label_text = f"{label}: {confidence:.2f}"

                # Put the label text on the frame
                cv2.putText(image, label_text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (64, 224, 208), 2)

                # Calculate the center of the bounding box
                bbox_center_x = (x1 + x2) // 2

                # Calculate the angle of the person relative to the camera
                person_angle = self.calculate_angle(bbox_center_x)

        # Publish the overlay image with bounding boxes
        overlay_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.overlay_publisher.publish(overlay_msg)

        # If a person is detected, find the distance and publish angle/distance
        if person_angle is not None and self.latest_laser_scan is not None:
            person_distance = self.get_distance_from_laser(person_angle)
            if person_distance is not None:
                self.publish_angle_distance(person_angle, person_distance)

    def calculate_angle(self, bbox_center_x):
        
        # Offset from the center of the image
        offset_from_center = bbox_center_x - (self.image_width / 2)

        # Calculate the angle using the camera's FOV
        angle = (offset_from_center / self.image_width) * self.camera_fov

        return angle

    def laser_callback(self, msg):
        
        self.latest_laser_scan = msg

    def get_distance_from_laser(self, angle):
        
        if self.latest_laser_scan is None:
            return None

        # Get the index corresponding to the angle
        laser_angle_min = self.latest_laser_scan.angle_min
        laser_angle_max = self.latest_laser_scan.angle_max
        laser_angle_increment = self.latest_laser_scan.angle_increment

        # Convert angle to radians
        angle_rad = np.deg2rad(angle)

        if laser_angle_min <= angle_rad <= laser_angle_max:
            # Calculate the index in the ranges array
            index = int((angle_rad - laser_angle_min) / laser_angle_increment)
            return self.latest_laser_scan.ranges[index]
        else:
            return None

    def publish_angle_distance(self, angle, distance):
        
        msg = Float32MultiArray()
        msg.data = [angle, distance]

        print("Publishing angle/distance: ", msg.data)
        self.angle_distance_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PersonDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
