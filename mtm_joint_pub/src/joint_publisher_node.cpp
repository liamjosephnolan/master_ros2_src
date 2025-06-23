#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

float x_offset = 0.2625;
float y_offset = 0.24053;
float z_offset = 0.0525;

class JointPublisherNode : public rclcpp::Node
{
public:
    JointPublisherNode()
        : Node("joint_publisher_node"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        // Publishers
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 100);
        target_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/model_pose", 100); // Updated topic name

        // Subscriber for MTM joint states
        mtm_joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/mtm_joint_states", 10,
            std::bind(&JointPublisherNode::mtmJointStateCallback, this, std::placeholders::_1)
        );

        // Timer to regularly publish target pose
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&JointPublisherNode::publishTargetPose, this)
        );

        RCLCPP_INFO(this->get_logger(), "Node initialized: publishing joint states and /model_pose.");
    }

private:
    // Helper function to convert degrees to radians
    double degToRad(double degrees)
    {
        return degrees * M_PI / 180.0;
    }

    // Callback for MTM joint states
    void mtmJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            const std::string& joint_name = msg->name[i];
            double position = msg->position[i];

            if (joint_name == "right_joint_1") {
                right_J1_joint_ = degToRad(position);
            } else if (joint_name == "right_joint_2") {
                right_J2_joint_ = degToRad(position);
            } else if (joint_name == "right_joint_3") {
                right_J3_joint_ = degToRad(position);
            } else if (joint_name == "right_gimbal_3") {
                right_G3_joint_ = degToRad(position);
            } else if (joint_name == "right_gimbal_2") {
                right_G2_joint_ = degToRad(position);
            } else if (joint_name == "right_gimbal_1") {
                right_G1_joint_ = degToRad(position);
            } else if (joint_name == "right_gimbal_0") {
                right_G0_1_joint_ = degToRad(position);
                right_G0_2_joint_ = degToRad(position);
            }
        }

        publishJointState();
    }

    void publishJointState()
    {
        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = this->get_clock()->now();
        joint_state.name = {
            "right_J1_joint", "right_J2_joint", "right_J3_joint",
            "right_G3_joint", "right_G2_joint", "right_G1_joint",
            "right_G0_1_joint", "right_G0_2_joint"
        };
        joint_state.position = {
            right_J1_joint_, right_J2_joint_, right_J3_joint_,
            right_G3_joint_, right_G2_joint_, right_G1_joint_,
            right_G0_1_joint_, right_G0_2_joint_
        };

        joint_state_publisher_->publish(joint_state);
    }

    void publishTargetPose()
    {
        rclcpp::Time now = this->get_clock()->now();

        if (tf_buffer_.canTransform("base_link", "right_G1_link", tf2::TimePointZero, tf2::durationFromSec(1.0))) {
            try {
                auto transform_stamped = tf_buffer_.lookupTransform("base_link", "right_G1_link", tf2::TimePointZero);

                geometry_msgs::msg::PoseStamped target_pose;
                target_pose.header.stamp = now;
                target_pose.header.frame_id = "base_link";
                target_pose.pose.position.x = transform_stamped.transform.translation.x - x_offset;
                target_pose.pose.position.y = transform_stamped.transform.translation.y - y_offset;
                target_pose.pose.position.z = transform_stamped.transform.translation.z - z_offset;
                target_pose.pose.orientation = transform_stamped.transform.rotation;

                target_pose_publisher_->publish(target_pose);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
            }
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Transform from 'right_G1_link' to 'base_link' not yet available.");
        }
    }

    // Joint positions
    double right_J1_joint_ = 0.0;
    double right_J2_joint_ = 0.0;
    double right_J3_joint_ = 0.0;
    double right_G3_joint_ = 0.0;
    double right_G2_joint_ = 0.0;
    double right_G1_joint_ = 0.0;
    double right_G0_1_joint_ = 0.0;
    double right_G0_2_joint_ = 0.0;

    // ROS 2 communication objects
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_publisher_; // Updated topic name
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr mtm_joint_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    // TF2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
