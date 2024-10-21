#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"

class GasAreaMarker : public rclcpp::Node
{
public:
    GasAreaMarker() : Node("gas_area_marker")
    {
        // Publisher for gas area marker
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("gas_area_marker", 10);

        // Subscriber for TurtleBot's odometry data
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&GasAreaMarker::odom_callback, this, std::placeholders::_1));

        // Gas area center and radius
        gas_area_center_.x = 2.0;
        gas_area_center_.y = 2.0;
        gas_area_radius_ = 1.0;

        // Publish the marker for the gas area
        publish_gas_marker();
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Get the TurtleBot's current position
        double robot_x = msg->pose.pose.position.x;
        double robot_y = msg->pose.pose.position.y;

        // Check if the robot is within the gas area
        bool inside_gas_area = is_inside_gas_area(robot_x, robot_y);

        if (inside_gas_area)
        {
            RCLCPP_WARN(this->get_logger(), "TurtleBot is inside the gas area");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "TurtleBot is outside the gas area");
        }
    }

    bool is_inside_gas_area(double x, double y)
    {
        // Check if the distance from the robot to the gas center is less than the radius
        double dx = x - gas_area_center_.x;
        double dy = y - gas_area_center_.y;
        double distance = sqrt(dx * dx + dy * dy);

        return distance <= gas_area_radius_;
    }

    void publish_gas_marker()
    {
        // Create a marker to represent the gas area
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();

        marker.ns = "gas_area";
        marker.id = 0;

        marker.type = visualization_msgs::msg::Marker::CYLINDER; // Represent the gas area as a cylinder
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the position and scale (size) of the marker
        marker.pose.position.x = gas_area_center_.x;
        marker.pose.position.y = gas_area_center_.y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = gas_area_radius_ * 2;  // Diameter of the cylinder
        marker.scale.y = gas_area_radius_ * 2;  // Diameter of the cylinder
        marker.scale.z = 0.1;                   // Height of the cylinder (flat)

        // Set the color of the marker (red for gas area)
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;  // 80% transparency

        // Publish the marker
        marker_pub_->publish(marker);
    }

    // Member variables
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    geometry_msgs::msg::Point gas_area_center_;  // Center of the gas area
    double gas_area_radius_;                     // Radius of the gas area
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GasAreaMarker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
