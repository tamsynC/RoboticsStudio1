import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3


class Point:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0

class ScanInImgOut(Node):
    def __init__(self):
        super().__init__('ScanInImgOut')
        self.laserSub = self.create_subscription(LaserScan, '/scan', self.laserCallback, 10)
        self.odoSub = self.create_subscription(Odometry, '/odom', self.odoCallback, 10)
        
        self.laserSub
        self.odoSub
        
        self.objPub = self.create_publisher(Marker, '/cylinderPos', 10)
        
        self.odoReading = Odometry()
        
    def laserCallback(self, data):
        #take data in, find objects, push objects
        prevRange = np.inf
        count = 1
        
        inObject = False
        
        startPoint = Point()
        endPoint = Point()
        
        for range in data.ranges[1:]:
            #Need to prevent double infs from occuring
            #if range != np.nan and range != np.inf:

                
            if abs(prevRange - range) < 0.3:
                    #print("inObject", inObject)
                if range != np.inf or range != np.nan:
                    if inObject == False:
                        inObject = True

                        nearestAngle = data.angle_min + (count * data.angle_increment)

                        startPoint.x = range * np.cos(nearestAngle)
                        startPoint.y = range * np.sin(nearestAngle)
                        startPoint.z = 0.0
                    else:
                        nearestAngle = data.angle_min + (count * data.angle_increment)
                        
                        endPoint.x = range * np.cos(nearestAngle)
                        endPoint.y = range * np.sin(nearestAngle)
                        endPoint.z = 0.0 
            
                    #print("triggered the else", inObject)
            else:
                if inObject == True:
                    inObject = False
                
                    midPoint = Point()
                    midPoint.x = (startPoint.x + endPoint.x)/2
                    midPoint.y = (startPoint.y + endPoint.y)/2
                    midPoint.z = 0.0
                    
                    print("start pos:", startPoint.x, startPoint.y)
                    print("end pos:", endPoint.x, endPoint.y)
                    self.pushObject(midPoint)
                        
                    
            prevRange = range     
            count += 1
    
    def odoCallback(self, data):
        #store global odo data
        self.odoReading = data
        pass
    
    def pushObject(self, position):
        
        #print("MadeObject") 
        
        scale = Vector3()
        scale.x = 0.5
        scale.y = 0.5
        scale.z = 0.5
        
        cylinder = Marker()
        cylinder.header.frame_id = "base_link"
        cylinder.type = 3
        cylinder.ns = "cylinderObstacle"
        cylinder.id = 1
        cylinder.action = Marker.ADD
        cylinder.scale = scale
        cylinder.pose.position.x = position.x
        cylinder.pose.position.y = position.y
        cylinder.pose.position.z = position.z
        
        cylinder.color.r = 0.0
        cylinder.color.g = 1.0
        cylinder.color.b = 0.0
        cylinder.color.a = 1.0
        
        print("object pushed, pos: ", position.x, position.y)
        
        self.objPub.publish(cylinder)
        
        pass
    
def main(args=None):
    rclpy.init(args=args)

    SubPubs = ScanInImgOut()

    rclpy.spin(SubPubs) 

    rclpy.shutdown(SubPubs)


if __name__ == '__main__':
    main()