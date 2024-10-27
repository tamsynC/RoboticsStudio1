import rclpy
from rclpy import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry

class HQ(Node):
    def __init__(self):
        super.__init__("HQNode")
        self.gasSub = self.create_subscription(String, "/HQGas", self.gasCallback, 10)
        self.audioSub = self.create_subscription(String, "/HQAudio", self.audioCallback, 10)
        self.peopleSub = self.create_subscription(String, "/HQPeople", self.peopleCallback, 10)
        self.odoSub = self.create_subscription(Odometry, "/odom", self.odoCallback, 10)
        
        self.gasArray = []
        self.audioArray = []
        self.peopleArray = []
        
        self.odo = Odometry()
        #insert timer for statusLoop
        self.create_timer(2.0, self.StatusLoop)
        
    def odoCallback(self, msg):
        self.odo = msg

    def gasCallback(self, msg):
        DataAndLocation = (msg, self.odo)
        self.gasArray.append(DataAndLocation)

    def audioCallback(self, msg):
        DataAndLocation = (msg, self.odo)
        self.audioArray.append(DataAndLocation)
        
    def peopleCallback(self, msg):
        DataAndLocation = (msg, self.odo)
        self.peopleArray.append(DataAndLocation)
    
    def StatusLoop(self):
        
        #Insert status update to be called on a timer
        print("*** Latest Updates ***")
        if len(self.gasArray) > 0:
            latestDetection = self.gasArray[len(self.gasArray) - 1][0]
            latestPosition = self.gasArray[len(self.gasArray) - 1][1].pose.pose.position
            print("Latest gas detected:", latestDetection, "At Position: X:", latestPosition.x, "Y:", latestPosition.y)
        else:
            print("No harmful gases detected yet")
        
        if len(self.audioArray) > 0:
            latestDetection = self.audioArray[len(self.audioArray) - 1][0]
            latestPosition = self.audioArray[len(self.audioArray) - 1][1].pose.pose.position
            print("Latest audio detected:", latestDetection, "At Position: X:", latestPosition.x, "Y:", latestPosition.y)
        else:
            print("No loud noises detected yet")
        
        if len(self.audioArray) > 0:
            latestDetection = self.audioArray[len(self.audioArray) - 1][0]
            latestPosition = self.audioArray[len(self.audioArray) - 1][1].pose.pose.position
            print("Latest person detected:", latestDetection, "At Position: X:", latestPosition.x, "Y:", latestPosition.y)
        else:
            print("No people detected yet")
        print("**********************\n")

if __name__ == "__main__":
    #insert ros startup sequence
    rclpy.init(args=None)
    node = HQ()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    
    pass    
    