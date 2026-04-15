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
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #------------------------------------------------------------
        # Action client for NavigateToPose
        #------------------------------------------------------------
        self.navigate_to_pose_client =  ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.declare_parameter('goal_trigg', False)

        room_pose={
            'x': 0.261382,
            'y': 1.1084,
            'z': 0.0,
            'ox': 0.0,
            'oy': 0.0,
            'oz': 0.0863898,
            'ow': 0.996261
        }
        hallway_pose={
            'x': -6.34,
            'y': -0.1,
            'z': 0.0,
            'ox': 0.0,
            'oy': 0.0,
            'oz': 1.0,
            'ow': 0.1237
        }
        self.poses = [room_pose , hallway_pose]



    def timer_callback(self):
        gTrig = self.get_parameter('goal_trigg').get_parameter_value().bool_value
        self.get_logger().info('Checking for goal trigger...')
        if gTrig:
            self.set_parameters([Parameter('goal_trigg', Parameter.Type.BOOL, False)])
            pose = self.poses.pop(0)

            g_msg = NavigateToPose.Goal()
            g_msg.pose = PoseStamped()
            g_msg.pose.header.frame_id = 'map'
            g_msg.pose.header.stamp = self.get_clock().now().to_msg()
            g_msg.pose.pose.position.x = pose['x']
            g_msg.pose.pose.position.y = pose['y']
            g_msg.pose.pose.position.z = pose['z']
            g_msg.pose.pose.orientation.x = pose['ox']
            g_msg.pose.pose.orientation.y = pose['oy']
            g_msg.pose.pose.orientation.z = pose['oz']
            g_msg.pose.pose.orientation.w = pose['ow']

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