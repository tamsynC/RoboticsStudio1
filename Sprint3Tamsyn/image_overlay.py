import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np

class SlamOverlay(Node):
    def __init__(self):

        super().__init__('slam_overlay')

        # Subscribe to the SLAM map topic
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Load the Gazebo world map (static image)
        self.gazebo_map = cv2.imread('/home/tamsyn/RoboticsStudio1/Sprint3Tamsyn/maps/map_v4.png', cv2.IMREAD_COLOR)
        self.gazebo_map_shape = self.gazebo_map.shape

        # Display window
        cv2.namedWindow('SLAM Overlay', cv2.WINDOW_NORMAL)

    def map_callback(self, msg):
        # Convert the OccupancyGrid data to an OpenCV image
        slam_map = self.occupancy_grid_to_image(msg)

        # Resize the SLAM map to match the Gazebo map size
        slam_map_resized = cv2.resize(slam_map, (self.gazebo_map_shape[1], self.gazebo_map_shape[0]))

        # Convert SLAM map to 3-channel to match the Gazebo map for overlay
        slam_map_colored = cv2.cvtColor(slam_map_resized, cv2.COLOR_GRAY2BGR)

        # Blend the maps together
        alpha = 0.5  # Transparency for SLAM map
        overlayed_map = cv2.addWeighted(slam_map_colored, alpha, self.gazebo_map, 1.0 - alpha, 0)

        # Display the overlayed map
        cv2.imshow('SLAM Overlay', overlayed_map)
        cv2.waitKey(1)  # Update the display

    def occupancy_grid_to_image(self, msg):
        # Convert the occupancy grid data to a numpy array
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))

        # Convert -1 values (unknown) to 127 (gray), 0 to 255 (white), and 100 to 0 (black)
        img = np.zeros((height, width), dtype=np.uint8)
        img[data == -1] = 127  # Unknown areas are gray
        img[data == 0] = 255   # Free space is white
        img[data == 100] = 0   # Occupied space is black

        return img

def main(args=None):
    rclpy.init(args=args)
    node = SlamOverlay()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
