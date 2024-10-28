import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3

class GasMarkerPublisher(Node):
    """
    A ROS 2 node that publishes markers for gas types in RViz and checks proximity of TurtleBot.
    """
    def __init__(self):
        super().__init__('gas_marker_publisher')

        # Publisher for the markers
        self.marker_publisher = self.create_publisher(Marker, '/gas_markers', 10)
        
        # Timer to publish markers every second
        self.create_timer(1.0, self.publish_markers)
        
        # Subscriber for TurtleBot's position (odometry)
        self.create_subscription(
            Odometry,
            'odom',  # Ensure this matches your TurtleBot's odometry topic
            self.odom_callback,
            10
        )

        # Define marker positions in the map frame
        self.marker_positions = {
            "CH4": Point(x=8.0, y=1.0, z=0.0),
            "CO2": Point(x=-6.0, y=1.0, z=0.0),
            "CO": Point(x=-2.0, y=-3.0, z=0.0),
        }

        # Set the distance thresholds for proximity check
        self.distance_thresholds = {
            "low": 3.0,    # 3 meters
            "high": 2.0,   # 2 meters
            "extreme": 1.0 # 1 meter
        }

        # Track the last known concentration levels for each marker
        self.last_concentration_levels = {name: "none" for name in self.marker_positions.keys()}

    def publish_markers(self):
        """
        Publishes markers for gas types CH4, CO2, and CO in RViz.
        """
        marker_data = [
            {"name": "CH4", "color": (1.0, 0.0, 0.0), "position": self.marker_positions["CH4"]},  # Red
            {"name": "CO2", "color": (0.0, 1.0, 0.0), "position": self.marker_positions["CO2"]},  # Green
            {"name": "CO",  "color": (0.0, 0.0, 1.0), "position": self.marker_positions["CO"]},   # Blue
        ]
        
        scale = Vector3(x=0.5, y=0.5, z=1.0)  # Marker scale

        for idx, data in enumerate(marker_data):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.CYLINDER
            marker.ns = data["name"]
            marker.id = idx
            marker.action = Marker.ADD
            marker.pose.position = data["position"]
            marker.scale = scale
            marker.color.r = data["color"][0]
            marker.color.g = data["color"][1]
            marker.color.b = data["color"][2]
            marker.color.a = 1.0  # Fully opaque
            
            # Publish the marker
            self.marker_publisher.publish(marker)

    def odom_callback(self, msg):
        """
        Callback for the TurtleBot's odometry data to check proximity to markers.
        """
        robot_position = msg.pose.pose.position
        
        for name, marker_position in self.marker_positions.items():
            distance = self.calculate_distance(robot_position, marker_position)
            new_concentration_level = self.determine_concentration_level(distance)
            
            # Only log if the concentration level has changed
            if new_concentration_level != self.last_concentration_levels[name]:
                self.last_concentration_levels[name] = new_concentration_level
                self.get_logger().info(f"{new_concentration_level} concentration of {name} detected!")

    def determine_concentration_level(self, distance):
        """
        Determine the concentration level based on the distance.
        """
        if distance <= self.distance_thresholds["extreme"]:
            return "Extreme"
        elif distance <= self.distance_thresholds["high"]:
            return "High"
        elif distance <= self.distance_thresholds["low"]:
            return "Low"
        else:
            return "None"

    def calculate_distance(self, p1, p2):
        """
        Calculate the Euclidean distance between two points.
        """
        return ((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2) ** 0.5

def main(args=None):
    rclpy.init(args=args)
    
    # Initialize the GasMarkerPublisher node
    node = GasMarkerPublisher()
    
    # Keep the node running
    rclpy.spin(node)
    
    # Shutdown the node after use
    rclpy.shutdown()

if __name__ == '__main__':
    main()
