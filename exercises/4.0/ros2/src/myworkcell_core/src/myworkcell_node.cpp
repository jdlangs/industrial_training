#include <rclcpp/rclcpp.hpp>
#include <myworkcell_core/srv/localize_part.hpp>

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

class ScanNPlan : public rclcpp::Node
{
public:
  ScanNPlan() : Node("scan_n_plan", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    if (! this->has_parameter("base_frame"))
    {
      this->declare_parameter("base_frame", "world");
    }

    vision_client_ = this->create_client<myworkcell_core::srv::LocalizePart>("localize_part");
    trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "fake_joint_trajectory_controller/joint_trajectory", rclcpp::QoS(1));
  }

  void start(const std::string& base_frame)
  {
    RCLCPP_INFO(get_logger(), "Attempting to localize part");

    // Create a request for the LocalizePart service call
    auto request = std::make_shared<myworkcell_core::srv::LocalizePart::Request>();
    request->base_frame = base_frame;

    auto future = vision_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to receive LocalizePart service response");
      return;
    }

    auto response = future.get();
    if (! response->success)
    {
      RCLCPP_ERROR(this->get_logger(), "LocalizePart service failed");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Part Localized:  x: %f, y: %f, z: %f",
        response->pose.position.x,
        response->pose.position.y,
        response->pose.position.z);

    geometry_msgs::msg::PoseStamped move_target;
    move_target.header.frame_id = base_frame;
    move_target.pose = response->pose;

    auto moveit_cpp = std::make_shared<moveit::planning_interface::MoveItCpp>(this->shared_from_this());
    moveit::planning_interface::PlanningComponent arm("manipulator", moveit_cpp);

    arm.setGoal(move_target, "tool0");

    RCLCPP_INFO(this->get_logger(), "Generating motion plan to target pose");
    const auto plan_solution = arm.plan();

    RCLCPP_INFO(this->get_logger(), "Sending trajectory for execution");
    if (plan_solution)
    {
      moveit_msgs::msg::RobotTrajectory robot_trajectory;
      plan_solution.trajectory->getRobotTrajectoryMsg(robot_trajectory);
      trajectory_pub_->publish(robot_trajectory.joint_trajectory);
    }
  }

private:
  // Planning components
  rclcpp::Client<myworkcell_core::srv::LocalizePart>::SharedPtr vision_client_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
};

int main(int argc, char **argv)
{
  // This must be called before anything else ROS-related
  rclcpp::init(argc, argv);

  // Create the ScanNPlan node
  auto app = std::make_shared<ScanNPlan>();

  std::string base_frame;
  app->get_parameter("base_frame", base_frame);

  //Wait for the vision node to receive data
  rclcpp::sleep_for(std::chrono::seconds(2));

  app->start(base_frame);
  rclcpp::spin(app);

  rclcpp::shutdown();
  return 0;
}
