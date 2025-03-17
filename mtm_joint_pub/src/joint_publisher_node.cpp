#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <regex>
#include <cmath>  // Include cmath for M_PI and conversions

class JointPublisherNode : public rclcpp::Node
{
public:
    JointPublisherNode() : Node("joint_publisher_node")
    {   
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        joint_state_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/mtm_joint_states", 10, 
            std::bind(&JointPublisherNode::jointStateCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Listening to /mtm_joint_states and publishing joint_states.");
    }

private:
    void jointStateCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::regex joint_regex(R"(joint1_angle:\s*([\d\.\-]+),\s*joint2_angle:\s*([\d\.\-]+),\s*joint3_angle:\s*([\d\.\-]+))");
        std::smatch match;

        if (std::regex_search(msg->data, match, joint_regex) && match.size() == 4)
        {
            // Convert degrees to radians before updating joint values
            shoulder_joint_ = degToRad(std::stod(match[1].str()));  // joint1 -> shoulder
            upper_joint_ = degToRad(std::stod(match[2].str()));     // joint2 -> upper
            lower_joint_ = degToRad(std::stod(match[3].str()));     // joint3 -> lower

            publishJointState();
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Failed to parse joint states: %s", msg->data.c_str());
        }
    }

    void publishJointState()
    {
        auto joint_state = sensor_msgs::msg::JointState();
        joint_state.header.stamp = this->get_clock()->now();
        joint_state.name = {"shoulder_joint", "upper_joint", "lower_join", "gimbal_joint", "left_lever_joint", "right_lever_joint"};
        joint_state.position = {shoulder_joint_, upper_joint_, lower_joint_, gimbal_joint_, left_lever_joint_, right_lever_joint_};

        joint_state_publisher_->publish(joint_state);
    }

    // Helper function: Convert degrees to radians
    double degToRad(double degrees)
    {
        return degrees * M_PI / 180.0;
    }

    // Joint position variables
    double shoulder_joint_ = 0.0;
    double upper_joint_ = 0.0;
    double lower_joint_ = 0.0;
    double gimbal_joint_ = 0.0;
    double left_lever_joint_ = 0.0;
    double right_lever_joint_ = 0.0;

    // ROS 2 objects
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr joint_state_subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
