#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class Phase1TrigVint(Node):
    def __init__(self):
        super().__init__('p1_trig_vint_node')
        timer_period = 1.0  # seconds

        self.subscription = self.create_subscription(
            String,
            '/command', 
            self.listener_callback,
            10
        )

        #------------------------------------------------------------
        # Action client for NavigateToPose
        #------------------------------------------------------------
        self.navigate_to_pose_client =  ActionClient(self, NavigateToPose, 'navigate_to_pose')
        #self.declare_parameter('goal_trigg', False)


    def listener_callback(self,msg):
            # Send a navigation goal to Nav2 when the message "go" is received
            if not "table" in msg.data:
                self.get_logger().info(f"Received command: {msg.data}, ignoring.")
                return
            g_msg = NavigateToPose.Goal()
            g_msg.pose = PoseStamped()
            g_msg.pose.header.frame_id = 'map'
            g_msg.pose.header.stamp = self.get_clock().now().to_msg()
            g_msg.pose.pose.position.x = -0.6
            g_msg.pose.pose.position.y = 0.95
            g_msg.pose.pose.position.z = 0.0
            g_msg.pose.pose.orientation.x = 0.0
            g_msg.pose.pose.orientation.y = 0.0
            g_msg.pose.pose.orientation.z = -0.223333
            g_msg.pose.pose.orientation.w = 0.974742

            future = self.navigate_to_pose_client.send_goal_async(g_msg)
            future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')


def main():
    rclpy.init()
    node = Phase1TrigVint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()