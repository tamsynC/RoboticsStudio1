# AUDIO NODE USED for publisher and subscribers
# https://github.com/mgonzs13/audio_common

# AUDIO sources https://pixabay.com/sound-effects/search/explosion/
# https://librosa.org/doc/latest/feature.html
# https://pypi.org/project/noisereduce/
# sudo apt install ffmpeg


# compare audio from subscriber to sampled audio

# play audio after set amount of time or when turtlebot reaches location



import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3


class ScanInImgOut(Node):
    """
    @brief A ROS2 node that processes LaserScan data and publishes detected objects.
    """
    def __init__(self):
        """
        @brief Initializes the ScanInImgOut node, setting up subscribers and publishers.
        """
        super().__init__('ScanInImgOut')
        self.laserSub = self.create_subscription(LaserScan, '/scan', self.laserCallback, 10)
        self.odoSub = self.create_subscription(Odometry, '/odom', self.odoCallback, 10)
        self.objPub = self.create_publisher(Marker, '/cylinderPos', 10)
        self.odoReading = Odometry()


    
    def odoCallback(self, data):

        self.odoReading = data
    
    def pushObject(self, position):

        scale = Vector3()
        scale.x = 0.3
        scale.y = 0.3
        scale.z = 0.3
        
        cylinder = Marker()
        cylinder.header.frame_id = "base_link"
        cylinder.type = 3
        cylinder.ns = "cylinderObstacle"
        cylinder.id = 1
        cylinder.action = Marker.ADD
        cylinder.scale = scale
        cylinder.pose.position.x = position.x+0.15
        cylinder.pose.position.y = position.y-0.15
        cylinder.pose.position.z = position.z
        
        cylinder.color.r = 0.0
        cylinder.color.g = 1.0
        cylinder.color.b = 0.0
        cylinder.color.a = 1.0
        
        self.objPub.publish(cylinder)

def main(args=None):
    """
    @brief Main function to initialize and run the ScanInImgOut node.
    """
    rclpy.init(args=args)
    SubPubs = ScanInImgOut()
    rclpy.spin(SubPubs)
    rclpy.shutdown(SubPubs)

if __name__ == '__main__':
    main()

