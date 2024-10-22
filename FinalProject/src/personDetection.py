# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, LaserScan
# from nav_msgs.msg import Odometry  # To get odometry data
# from std_msgs.msg import String  # To publish the result as a string
# from geometry_msgs.msg import Point

# from ultralytics import YOLO
# import cv2
# from cv_bridge import CvBridge
# import numpy as np
# import math

# class PersonDetector(Node):
#     def __init__(self):
#         super().__init__('person_detector')
        
#         # YOLOv8 model from Ultralytics
#         self.model = YOLO('yolov8n.pt')  # Use 'yolov8n.pt' or any other pretrained model
#         self.bridge = CvBridge()

#         # Publishers and subscribers
#         self.camera_subscriber = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
#         # self.laser_subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
#         # self.marker_publisher = self.create_publisher(Marker, '/person_marker', 10)
#         self.overlay_publisher = self.create_publisher(Image, '/camera/image_overlay', 10)

#         # Variables for storing data
#         # self.latest_laser_scan = None
#         # self.people_positions = []

#     def image_callback(self, msg):
#         # Convert image from ROS to OpenCV
#         frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

#         # Run YOLO detection
#         #can remove show=True used for testing
#         results = self.model(frame, show=True)

#         annotated_image = results[0].plot()  # Use plot() to annotate image with bounding boxes

#         # Publish overlay image
#         overlay_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
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
from nav_msgs.msg import Odometry  # To get odometry data
from std_msgs.msg import String  # To publish the result as a string
from geometry_msgs.msg import Point
from ultralytics import YOLO
import cv2
from cv_bridge import CvBridge
import numpy as np
import math

class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')
        
        # YOLOv8 model from Ultralytics
        self.model = YOLO('yolov8n.pt')  # Use 'yolov8n.pt' or any other pretrained model
        self.bridge = CvBridge()

        # Publishers and subscribers
        self.camera_subscriber = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.laser_subscriber = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.position_publisher = self.create_publisher(String, '/person_position', 10)
        self.overlay_publisher = self.create_publisher(Image, '/camera/image_overlay', 10)

        # Variables for storing data
        self.latest_laser_scan = None
        self.latest_odom = None
        self.people_positions = []

    # def image_callback(self, msg):
    #     # Convert image from ROS to OpenCV
    #     frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    #     # Run YOLO detection
    #     results = self.model(frame, show=True)
    #     detected_people = []

    #     for result in results:
    #         for box in result.boxes:
    #             # Assuming class 0 is 'person'
    #             if box.cls == 0:
    #                 bbox = box.xyxy  # Bounding box
    #                 detected_people.append(bbox)

    #     # Store detected people for laser scan callback
    #     self.people_positions = detected_people

    #     # Annotate image and publish overlay
    #     annotated_image = results[0].plot()
    #     overlay_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
    #     self.overlay_publisher.publish(overlay_msg)

    def image_callback(self, msg):
        # Convert image from ROS to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    
        # Run YOLO detection
        results = self.model(frame)
        detected_people = []
    
        for result in results:
            for box in result.boxes:
                # Assuming class 0 is 'person'
                if box.cls == 0:
                    bbox = box.xyxy  # Bounding box
    
                    # Ensure the bounding box has 4 valid values
                    if len(bbox) != 4:
                        self.get_logger().warn(f"Invalid bounding box: {bbox}")
                        continue
                    
                    # Check if the values are within the frame dimensions
                    x1, y1, x2, y2 = bbox.tolist()
                    if x1 < 0 or y1 < 0 or x2 > frame.shape[1] or y2 > frame.shape[0]:
                        self.get_logger().warn(f"Bounding box out of image bounds: {bbox}")
                        continue
                    
                    detected_people.append(bbox)
    
        # Store detected people for laser scan callback
        self.people_positions = detected_people
    
        # Annotate image and publish overlay
        annotated_image = results[0].plot()
        overlay_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
        self.overlay_publisher.publish(overlay_msg)


    def laser_callback(self, msg):
        self.latest_laser_scan = msg
        self.calculate_person_position()

    def odom_callback(self, msg):
        self.latest_odom = msg

    def calculate_person_position(self):
        if self.latest_laser_scan and self.latest_odom and self.people_positions:
            for bbox in self.people_positions:
                

                if bbox.shape != (4,):
                    self.get_logger().warn(f"Invalid bounding box: {bbox}")
                    continue

                # Approximate the range to the person based on LaserScan data
                # Use center of the bounding box to estimate position
                x1, y1, x2, y2 = bbox.tolist()
                center_x = (x1 + x2) / 2

                # Map the center_x to a corresponding angle in LaserScan data
                scan_angle = self.map_bbox_to_laser_angle(center_x)

                # Get the distance to the person from the LaserScan
                distance = self.latest_laser_scan.ranges[int(scan_angle)]

                # Calculate the person's position relative to the robot using odometry data
                person_position = self.get_person_global_position(scan_angle, distance)

                # Publish the position as a string
                self.publish_position(person_position)

    def map_bbox_to_laser_angle(self, center_x):
        # Assuming the field of view of the camera and the LaserScan overlaps
        # and that center_x can be mapped to an index in the LaserScan array
        laser_index = int(center_x / 640 * len(self.latest_laser_scan.ranges))  # Assuming 640px image width
        return laser_index

    def get_person_global_position(self, scan_angle, distance):
        # Get the robot's position and orientation from odometry
        robot_position = self.latest_odom.pose.pose.position
        robot_orientation = self.latest_odom.pose.pose.orientation

        # Convert orientation (quaternion) to Euler angles (yaw for 2D rotation)
        yaw = self.quaternion_to_yaw(robot_orientation)

        # Calculate the person's position relative to the robot
        person_x = robot_position.x + distance * math.cos(yaw + scan_angle)
        person_y = robot_position.y + distance * math.sin(yaw + scan_angle)

        # Create a Point to store the person's position
        person_position = Point()
        person_position.x = person_x
        person_position.y = person_y
        person_position.z = 0.0

        return person_position

    def quaternion_to_yaw(self, orientation):
        # Convert a quaternion to a yaw angle (rotation around z-axis)
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def publish_position(self, position):
        position_str = f"Person at: x={position.x:.2f}, y={position.y:.2f}"
        self.get_logger().info(position_str)
        position_msg = String()
        position_msg.data = position_str
        self.position_publisher.publish(position_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PersonDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
