import rclpy
from rclpy import Node
from std_msgs.msg import String

class HQ(Node):
    def __init__(self):
        super.__init__("HQNode")
        self.gasSub = self.create_subscription(String, "/HQGas", self.gasCallback, 10)
        self.audioSub = self.create_subscription(String, "/HQAudio", self.audioCallback, 10)
        self.peopleSub = self.create_subscription(String, "/HQPeople", self.peopleCallback, 10)
        
        self.gasStatus = "No Gas Reading Recorded"
        self.audioStatus = "No Audio Reading Detected"
        self.peopleStatus = "No Persons Detected"
        
        #insert timer for statusLoop
        

    def gasCallback(self, msg):
        self.gasStatus = msg.data

    def audioCallback(self, msg):
        self.audioStatus = msg.data
        
    def peopleCallback(self, msg):
        self.peopleStatus = msg.data
    
    def StatusLoop(self):
        
        #Insert status update to be called on a timer
        pass

if __name__ == "__main__":
    #insert ros startup sequence
    pass    
    