#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from action_msgs.srv import CancelGoal


class GOALtoNAV2(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        self._goal_handle = None

        self.subscription = self.create_subscription(
            Marker,
            "/select_poses",
            self.send_goal,
            10
        )
    
        #------------------------------------------------------------
        # Action client for NavigateToPose
        #------------------------------------------------------------
        self.navigate_to_pose_client =  ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._cancel_rviz_goals_client = self.create_client(CancelGoal, 'navigate_to_pose/_action/cancel_goal')


    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def send_goal(self,g_msg_rviz):
            
            # Making goal message
            self.get_logger().info('Cancel command received.')

            pose_msg = PoseStamped()

            pose_msg.header = g_msg_rviz.header  # Use the same frame and timestamp as the marker
            pose_msg.pose = g_msg_rviz.pose  # Copy the pose from the marker

            g_msg = NavigateToPose.Goal()
            g_msg.pose = pose_msg

            # Sending message

            if self._goal_handle:
                self.get_logger().info('Attempting to cancel the current goal...')
                cancel_future = self._goal_handle.cancel_goal_async()           
                cancel_future.add_done_callback(lambda fut: self.cancel_done_callback(fut, g_msg))
            
            else:
                self.get_logger().info('Attempting to send goal')
                if g_msg:
                    future = self.navigate_to_pose_client.send_goal_async(g_msg)
                    future.add_done_callback(lambda fut: self.goal_response_callback)
                else:
                    self.get_logger().warn('Goal is empty')


    def cancel_done_callback(self,fut,g_msg):
        result = fut.result()
        if len(result.goals_canceling) > 0:
            self.get_logger().info('Goal successfully cancelled.')
            self._goal_handle = None
            if g_msg:
                future = self.navigate_to_pose_client.send_goal_async(g_msg)
                future.add_done_callback(self.goal_response_callback)
            else:
                self.get_logger().warn('Goal is empty')
        else:
                self.get_logger().warn('Goal cancel failed or no goal was active.')


    def goal_response_callback(self, future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Goal was rejected.')
                return

            self._goal_handle = goal_handle
            self.get_logger().info('Goal accepted and executing.')



def main():
    rclpy.init()

    node = GOALtoNAV2()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()