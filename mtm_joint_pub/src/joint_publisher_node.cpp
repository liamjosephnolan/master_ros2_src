#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
// No longer need PoseStamped for the output, but TF2 needs it for transforms
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/time.h>
// This header is still needed for TF2 to work with geometry_msgs
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <string>

// Your offsets remain the same
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
        // Publisher for robot joint angles
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 100);

        // --- CHANGE 1: The publisher for /model_pose now uses sensor_msgs::msg::JointState ---
        target_pose_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/model_pose", 10);

        // Subscriber for MTM joint states
        mtm_joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/mtm_joint_states", 10,
            std::bind(&JointPublisherNode::mtmJointStateCallback, this, std::placeholders::_1)
        );

        // --- CHANGE 2 (RECOMMENDED): Timer rate adjusted for stability (e.g., 50 Hz) ---
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1), // 20ms period = 50 Hz rate. A much safer rate.
            std::bind(&JointPublisherNode::publishAll, this)
        );

        RCLCPP_INFO(this->get_logger(), "Node initialized: publishing joint_states and /model_pose at a controlled rate.");
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
        // This callback now ONLY updates the internal state variables.
        // It does not publish anything directly, preventing message flooding.
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
        // The call to publishJointState() is removed from here.
    }

    void publishRobotJoints()
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

    // --- CHANGE 3: The main publishing logic is now in a single timer callback ---
    void publishAll()
    {
        // First, publish the robot's joint angles at the controlled rate.
        publishRobotJoints();

        // Second, get the transform and publish the cartesian pose in the new format.
        rclcpp::Time now = this->get_clock()->now();

        if (tf_buffer_.canTransform("base_link", "right_G1_link", tf2::TimePointZero, tf2::durationFromSec(0.05))) {
            try {
                auto transform_stamped = tf_buffer_.lookupTransform("base_link", "right_G1_link", tf2::TimePointZero);

                // --- CHANGE 4: Create a JointState message for the cartesian pose ---
                sensor_msgs::msg::JointState cartesian_pose_msg;
                cartesian_pose_msg.header.stamp = now;
                cartesian_pose_msg.header.frame_id = ""; // Per your example format

                // Set the names for the cartesian coordinates
                cartesian_pose_msg.name = {"x", "y", "z"};

                // Calculate and set the position values after applying offsets
                cartesian_pose_msg.position = {
                    transform_stamped.transform.translation.x - x_offset,
                    transform_stamped.transform.translation.y - y_offset,
                    transform_stamped.transform.translation.z - z_offset
                };

                // Fill velocity and effort with zeros as per your format
                cartesian_pose_msg.velocity = {0.0, 0.0, 0.0};
                cartesian_pose_msg.effort = {0.0, 0.0, 0.0};

                // Publish the newly formatted message
                target_pose_publisher_->publish(cartesian_pose_msg);

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
    // --- CHANGE 5: The member variable type is updated ---
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr target_pose_publisher_;
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