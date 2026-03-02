#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from action_msgs.srv import CancelGoal
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class GOALtoNAV2(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        self._goal_handle_ = None
        self.is_command_sent_ = False
        self.mrk= Marker()

        #------------------------------------------------------------
        # Subscribers
        #------------------------------------------------------------

        self.create_subscription(Marker,"/select_poses",self.send_goal,10)

        qos_pc = QoSProfile( reliability=QoSReliabilityPolicy.BEST_EFFORT,history=QoSHistoryPolicy.KEEP_LAST,depth=1) # ensuring the newest message is read
        self.create_subscription(PointCloud2, '/lidar_RAM_segm', self.plane_cloud_callback, qos_pc)

        self.create_subscription(Marker, "/table_marker_rel",lambda msg:self.cb_marker(msg,"table"), 10)
        self.create_subscription(Marker, "/robot_marker_rel",lambda msg:self.cb_marker(msg,"robot"), 10)



        #------------------------------------------------------------
        # Publishers
        #------------------------------------------------------------
        self.pc_pub = self.create_publisher(PointCloud2, '/lidar_RAM_segm_rel',1)
        self.tmrk_pub = self.create_publisher(Marker, '/table_marker',1)
        self.rmrk_pub = self.create_publisher(Marker, '/robot_marker',1)
    
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

            if self._goal_handle_:
                self.get_logger().info('Attempting to cancel the current goal...')
                cancel_future = self._goal_handle_.cancel_goal_async()           
                cancel_future.add_done_callback(lambda fut: self.cancel_done_callback(fut, g_msg))
            
            else:
                self.get_logger().info('Attempting to send goal')
                if g_msg:

                    #Publishing the last marker in red to signal the table stops updating
                    self.is_command_sent_= True
                    self.mrk.color = type(self.mrk.color)(r=1.0, g=0.0, b=0.0, a=1.0)
                    self.tmrk_pub.publish(self.mrk)

                    future = self.navigate_to_pose_client.send_goal_async(g_msg)
                    future.add_done_callback(lambda fut: self.goal_response_callback)
                else:
                    self.get_logger().warn('Goal is empty')


    def cancel_done_callback(self,fut,g_msg):
        result = fut.result()
        if len(result.goals_canceling) > 0:
            self.get_logger().info('Goal successfully cancelled.')
            self._goal_handle_ = None
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

            self._goal_handle_ = goal_handle
            self.get_logger().info('Goal accepted and executing.')

    def plane_cloud_callback(self, plane_cloud_msg):
        if not self.is_command_sent_:
              self.pc_pub.publish(plane_cloud_msg)

    def cb_marker(self, msg: Marker, label):
         
         if label is "robot":
              mrk_pub=self.rmrk_pub
         else:
              mrk_pub=self.tmrk_pub
              self.mrk=msg
         
         if not self.is_command_sent_:
              mrk_pub.publish(msg)
              

        



def main():
    rclpy.init()

    node = GOALtoNAV2()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()