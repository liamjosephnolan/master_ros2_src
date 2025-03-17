#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import argparse

class JointStatePublisherCli(Node):

    def __init__(self):
        super().__init__('joint_state_publisher_cli')

        # Create a publisher for the /joint_states topic
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

    def publish_joint_state(self, joint_names, joint_positions):
        """
        Publish a JointState message with the given joint names and positions.
        """
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'  # Replace with your frame ID if needed

        msg.name = joint_names
        msg.position = joint_positions
        msg.velocity = [0.0] * len(joint_names)  # Set velocities to 0
        msg.effort = [0.0] * len(joint_names)    # Set efforts to 0

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published JointState: {msg}')

def main(args=None):
    rclpy.init(args=args)

    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Publish joint states to /joint_states')
    parser.add_argument('--joints', nargs='+', required=True, help='List of joint names')
    parser.add_argument('--positions', nargs='+', type=float, required=True, help='List of joint positions')
    args = parser.parse_args()

    # Ensure the number of joint names matches the number of positions
    if len(args.joints) != len(args.positions):
        raise ValueError("The number of joint names must match the number of positions.")

    # Create the node and publish the joint state
    node = JointStatePublisherCli()
    node.publish_joint_state(args.joints, args.positions)

    # Shutdown the node after publishing
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
