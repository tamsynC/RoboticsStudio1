/*
to send goal to robot the below code is needed.
To use, put into code as marked and call function with desired goal.

In the constructor:
nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

as a function:
void sendGoal(const geometry_msgs::msg::Pose &goal) {
    // Wait for the action server to be available
    while (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
    }

    nav2_msgs::action::NavigateToPose::Goal goalMsg;
    goalMsg.pose.header.frame_id = "map";
    goalMsg.pose.header.stamp = this->now();
    goalMsg.pose.pose.position.x = goal.position.x;
    goalMsg.pose.pose.position.y = goal.position.y;
    goalMsg.pose.pose.orientation.w = goal.orientation.w;
    goalMsg.pose.pose.orientation.x = goal.orientation.x;
    goalMsg.pose.pose.orientation.y = goal.orientation.y;
    goalMsg.pose.pose.orientation.z = goal.orientation.z;

    // Send the goal
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle) {
            //RCLCPP_INFO(this->get_logger(), "Goal accepted!");
      };

    send_goal_options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
            //RCLCPP_INFO(this->get_logger(), "Goal result received!");
        };

    auto goal_handle = nav_client_->async_send_goal(goalMsg, send_goal_options);
}
*/