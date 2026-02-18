import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Duration
import math

class MapBroadcaster(Node):
    def __init__(self):
        super().__init__('map_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.table_marker_pub = self.create_publisher(Marker, '/table_marker', 10)
        self.robot_marker_pub = self.create_publisher(Marker, '/robot_marker', 10)
        self.robot_pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1Hz


        # ---original_position---
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw_deg = 90.0
        self.robot_yaw_rad = math.radians(self.robot_yaw_deg)
        robot_name = "robot"

        self.table_x = 1.0
        self.table_y = 5.0
        self.table_yaw_deg = 0.0
        self.table_yaw_rad = math.radians(self.table_yaw_deg)
        table_name = "table"

    def yaw_to_quaternion(self, yaw):
        half = yaw * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def timer_callback(self):
        now = self.get_clock().now().to_msg()

        #---robot_TF---
        q_robot = self.yaw_to_quaternion(self.robot_yaw_rad)
        t_robot = TransformStamped()
        t_robot.header.stamp = now
        t_robot.header.frame_id = 'map'
        t_robot.child_frame_id = 'robot_base'
        t_robot.transform.translation.x = float(self.robot_x)
        t_robot.transform.translation.y = float(self.robot_y)
        t_robot.transform.translation.z = 0.0
        t_robot.transform.rotation.x = q_robot[0]
        t_robot.transform.rotation.y = q_robot[1]
        t_robot.transform.rotation.z = q_robot[2]
        t_robot.transform.rotation.w = q_robot[3]
        self.tf_broadcaster.sendTransform(t_robot)

        # ---table_TF---
        q_table = self.yaw_to_quaternion(self.table_yaw_rad)
        t_table = TransformStamped()
        t_table.header.stamp = now
        t_table.header.frame_id = 'map'
        t_table.child_frame_id = 'table'
        t_table.transform.translation.x = float(self.table_x)
        t_table.transform.translation.y = float(self.table_y)
        t_table.transform.translation.z = 0.0
        t_table.transform.rotation.x = q_table[0]
        t_table.transform.rotation.y = q_table[1]
        t_table.transform.rotation.z = q_table[2]
        t_table.transform.rotation.w = q_table[3]
        self.tf_broadcaster.sendTransform(t_table)

        # ---robot_PoseStamped---
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(self.robot_x)
        pose.pose.position.y = float(self.robot_y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q_robot[0]
        pose.pose.orientation.y = q_robot[1]
        pose.pose.orientation.z = q_robot[2]
        pose.pose.orientation.w = q_robot[3]
        self.robot_pose_pub.publish(pose)

        #---table_Marker---
        m_table = Marker()
        m_table.header.stamp = now
        m_table.header.frame_id = 'map'
        m_table.ns = 'table'
        m_table.id = 0
        m_table.type = Marker.CUBE
        m_table.action = Marker.ADD
        # tabel_position
        table_height = 0.75
        m_table.pose.position.x = float(self.table_x)
        m_table.pose.position.y = float(self.table_y)
        m_table.pose.position.z = table_height / 2.0
        m_table.pose.orientation.x = q_table[0]
        m_table.pose.orientation.y = q_table[1]
        m_table.pose.orientation.z = q_table[2]
        m_table.pose.orientation.w = q_table[3]
        # table_size
        m_table.scale.x = 1.2
        m_table.scale.y = 0.8
        m_table.scale.z = table_height
        m_table.color.r = 0.0
        m_table.color.g = 0.0
        m_table.color.b = 1.0
        m_table.color.a = 1.0
        m_table.lifetime = Duration(sec=0, nanosec=0)
        self.table_marker_pub.publish(m_table)

        # ---robot_Marker---
        m_robot = Marker()
        m_robot.header.stamp = now
        m_robot.header.frame_id = 'map'
        m_robot.ns = 'robot'
        m_robot.id = 1
        m_robot.type = Marker.CUBE
        m_robot.action = Marker.ADD
        # robot_position
        robot_height = 1.0
        m_robot.pose.position.x = float(self.robot_x)
        m_robot.pose.position.y = float(self.robot_y)
        m_robot.pose.position.z = robot_height / 2.0
        # robot_orientation
        m_robot.pose.orientation.x = q_robot[0]
        m_robot.pose.orientation.y = q_robot[1]
        m_robot.pose.orientation.z = q_robot[2]
        m_robot.pose.orientation.w = q_robot[3]
        # robot_size
        m_robot.scale.x = 0.5
        m_robot.scale.y = 0.5
        m_robot.scale.z = robot_height
        m_robot.color.r = 0.0
        m_robot.color.g = 1.0
        m_robot.color.b = 0.0
        m_robot.color.a = 1.0
        m_robot.lifetime = Duration(sec=0, nanosec=0)
        self.robot_marker_pub.publish(m_robot)


def main(args=None):
    rclpy.init(args=args)
    node = MapBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

