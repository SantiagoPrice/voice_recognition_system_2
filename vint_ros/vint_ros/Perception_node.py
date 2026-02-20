#!/usr/bin/env python3

import json
import vosk
import sounddevice as sd
import queue
import threading
import rclpy
import requests
import time
import numpy as np
import pyttsx3
import math
import random
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Empty
from rclpy.node import Node
from rclpy.parameter import Parameter as RclpyParameter
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import Parameter as MsgParameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from vosk import Model, KaldiRecognizer , GpuInit
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped, PoseStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from std_msgs.msg import String

#---------------------------------------------------
table_x = 0.0
table_y = 0.0
table_yaw_deg = 270.0
table_name = "table"
table_size_x = 1.5
table_size_y = 0.9
table_size_z = 0.75
table_thick=0.07

leg_width=0.1


robot_x = 3.0
robot_y = 2.0
robot_yaw_deg = 0.0
robot_name = "robot"
robot_size_x = 0.5
robot_size_y = 0.5
robot_size_z = 1.0

THETA_A = 15.0

table_hsize_x = table_size_x / 2
table_hsize_y = table_size_y / 2
corners = [
        (((table_x+table_hsize_x)*math.cos(table_yaw_deg)), ((table_y+table_hsize_y)*(-math.sin(table_yaw_deg))), table_size_z),
        (((table_x-table_hsize_x)*math.cos(table_yaw_deg)), ((table_y+table_hsize_y)*(-math.sin(table_yaw_deg))), table_size_z),
        (((table_x-table_hsize_x)*math.cos(table_yaw_deg)), ((table_y-table_hsize_y)*(-math.sin(table_yaw_deg))), table_size_z),
        (((table_x+table_hsize_x)*math.cos(table_yaw_deg)), ((table_y-table_hsize_y)*(-math.sin(table_yaw_deg))), table_size_z)
        ]

robot_yaw_rad = math.radians(robot_yaw_deg)
Robot = np.array([robot_x, robot_y, robot_yaw_deg, robot_yaw_rad, robot_name]) 
table_yaw_rad = math.radians(table_yaw_deg)
Table = np.array([table_x, table_y, table_yaw_deg, table_yaw_rad, table_name]) 

#----------------------------------------------------
class MapBroadcaster(Node):
    def __init__(self):
        super().__init__('map_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        

        #---- publish ----#
        self.table_marker_pub = self.create_publisher(Marker, '/table_marker', 10)
        self.robot_marker_pub = self.create_publisher(Marker, '/robot_marker', 10)
        self.RtoT_marker_pub = self.create_publisher(Marker, '/RtoT_marker', 10)
        self.Tcon_marker_pub = self.create_publisher(Marker, '/Tcon_marker', 10)
        self.Redge_marker_pub = self.create_publisher(Marker, '/Redge_marker', 10)
        self.robot_pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.marker = self.create_timer(1.0, self.publish_marker)        
        #---- subscribe ----#
        self.map_update_sub = self.create_subscription(
                Float32MultiArray,
                '/map_update',
                self.map_update_cb,
                10
                )

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def yaw_to_quaternion(self, yaw):
        half = yaw * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def map_update_cb(self, msg):
        self.get_logger().info("map_update")
        global robot_x
        global robot_y
        global robot_yaw_deg
        global robot_yaw_rad
        
        robot_x, robot_y, robot_yaw_deg, self.dummy_value = msg.data
        robot_yaw_rad = math.radians(robot_yaw_deg)

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def publish_marker(self):
        now = self.get_clock().now().to_msg()
        #---robot_TF---
        q_robot = self.yaw_to_quaternion(robot_yaw_rad)
        t_robot = TransformStamped()
        t_robot.header.stamp = now
        t_robot.header.frame_id = 'map'
        t_robot.child_frame_id = 'robot_base'
        t_robot.transform.translation.x = float(robot_x)
        t_robot.transform.translation.y = float(robot_y)
        t_robot.transform.translation.z = 0.0
        t_robot.transform.rotation.x = q_robot[0]
        t_robot.transform.rotation.y = q_robot[1]
        t_robot.transform.rotation.z = q_robot[2]
        t_robot.transform.rotation.w = q_robot[3]
        self.tf_broadcaster.sendTransform(t_robot)

        # ---table_TF---
        q_table = self.yaw_to_quaternion(table_yaw_rad)
        t_table = TransformStamped()
        t_table.header.stamp = now
        t_table.header.frame_id = 'map'
        t_table.child_frame_id = 'table'
        t_table.transform.translation.x = float(table_x)
        t_table.transform.translation.y = float(table_y)
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
        pose.pose.position.x = float(robot_x)
        pose.pose.position.y = float(robot_y)
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
        table_height = table_size_z
        m_table.pose.position.x = float(table_x)
        m_table.pose.position.y = float(table_y)
        m_table.pose.position.z = table_height-table_thick/2#table_height / 2.0
        m_table.pose.orientation.x = q_table[0]
        m_table.pose.orientation.y = q_table[1]
        m_table.pose.orientation.z = q_table[2]
        m_table.pose.orientation.w = q_table[3]
        # table_size
        m_table.scale.x = table_size_x
        m_table.scale.y = table_size_y
        m_table.scale.z = table_thick

        m_table.color.r = 0.0
        m_table.color.g = 0.0
        m_table.color.b = 1.0
        m_table.color.a = 0.5
        m_table.lifetime = Duration(sec=0, nanosec=0)
        self.table_marker_pub.publish(m_table)

        #---table legs_Marker---
        m_legs = Marker()
        m_legs.header.stamp = now
        m_legs.header.frame_id = 'map'
        m_legs.ns = 'table_legs'
        m_legs.id = 0
        m_legs.type = Marker.CUBE_LIST
        m_legs.action = Marker.ADD
        # first_leg_position
        table_height = table_size_z
        m_legs.pose.position.x = float(table_x)+float(table_x)/2#-leg_width
        m_legs.pose.position.y = float(table_y)+float(table_y)/2#-leg_width
        m_legs.pose.position.z = (table_height-table_thick)/2#table_height / 2.0
        m_legs.pose.orientation.x = q_table[0]
        m_legs.pose.orientation.y = q_table[1]
        m_legs.pose.orientation.z = q_table[2]
        m_legs.pose.orientation.w = q_table[3]
        # leg_size
        m_legs.scale.x = leg_width
        m_legs.scale.y = leg_width
        m_legs.scale.z = (table_height-table_thick)

        # leg separation

        # Calculate leg positions based on table size and leg separation
        leg_separation_x = (table_size_x - leg_width) / 2
        leg_separation_y = (table_size_y - leg_width) / 2

        # Four corners for legs
        leg_positions = [
            Point(x=table_x + leg_separation_x, y=table_y + leg_separation_y, z=0.0),
            Point(x=table_x - leg_separation_x, y=table_y + leg_separation_y, z=0.0),
            Point(x=table_x - leg_separation_x, y=table_y - leg_separation_y, z=0.0),
            Point(x=table_x + leg_separation_x, y=table_y - leg_separation_y, z=0.0),
        ]
        m_legs.points.extend(leg_positions)
        m_legs.color.r = 0.0
        m_legs.color.g = 0.0
        m_legs.color.b = 1.0
        m_legs.color.a = 1.0
        m_legs.lifetime = Duration(sec=0, nanosec=0)
        self.table_marker_pub.publish(m_legs)

        # ---robot_Marker---
        m_robot = Marker()
        m_robot.header.stamp = now
        m_robot.header.frame_id = 'map'
        m_robot.ns = 'robot'
        m_robot.id = 1
        m_robot.type = Marker.CUBE
        m_robot.action = Marker.ADD
        # robot_position
        robot_height = robot_size_z
        m_robot.pose.position.x = float(robot_x)
        m_robot.pose.position.y = float(robot_y)
        m_robot.pose.position.z = robot_height / 2.0
        # robot_orientation
        m_robot.pose.orientation.x = q_robot[0]
        m_robot.pose.orientation.y = q_robot[1]
        m_robot.pose.orientation.z = q_robot[2]
        m_robot.pose.orientation.w = q_robot[3]
        # robot_size
        m_robot.scale.x = robot_size_x
        m_robot.scale.y = robot_size_y
        m_robot.scale.z = robot_height
        m_robot.color.r = 0.0
        m_robot.color.g = 1.0
        m_robot.color.b = 0.0
        m_robot.color.a = 1.0
        m_robot.lifetime = Duration(sec=0, nanosec=0)
        self.robot_marker_pub.publish(m_robot)
        
        # ---vector_P_ijkl---
        m_Tcon = Marker()
        m_Tcon.header.stamp = now
        m_Tcon.header.frame_id = "map"
        m_Tcon.ns = "line"
        m_Tcon.id = 2
        m_Tcon.type = Marker.LINE_LIST
        m_Tcon.action = Marker.ADD
        m_Tcon.scale.x = 0.02
        m_Tcon.color.r = 0.8
        m_Tcon.color.g = 0.3
        m_Tcon.color.b = 0.0
        m_Tcon.color.a = 1.0
        for corner in corners:
            start = Point()
            start.x, start.y, start.z = table_x, table_y, (table_size_z)
            end = Point()
            end.x, end.y, end.z = corner
            m_Tcon.points.extend([start, end])
        m_Tcon.lifetime = Duration(sec=0, nanosec=0)
        self.Tcon_marker_pub.publish(m_Tcon)

        # ---vector_RtoT---
        m_RtoT = Marker()
        m_RtoT.header.stamp = now
        m_RtoT.header.frame_id = "map"
        m_RtoT.ns = "line"
        m_RtoT.id = 2
        m_RtoT.type = Marker.LINE_LIST
        m_RtoT.action = Marker.ADD
        m_RtoT.scale.x = 0.02
        m_RtoT.color.r = 0.0
        m_RtoT.color.g = 0.8
        m_RtoT.color.b = 0.3
        m_RtoT.color.a = 1.0
        start_point_RtoT = Point()
        start_point_RtoT.x = robot_x
        start_point_RtoT.y = robot_y
        start_point_RtoT.z = robot_size_z
        end_point_RtoT = Point()
        end_point_RtoT.x = table_x
        end_point_RtoT.y = table_y
        end_point_RtoT.z = table_size_z
        m_RtoT.points.append(start_point_RtoT)
        m_RtoT.points.append(end_point_RtoT)
        m_RtoT.lifetime = Duration(sec=0, nanosec=0)
        self.RtoT_marker_pub.publish(m_RtoT)

def main():
    rclpy.init()
    broadcaster = MapBroadcaster()
    rclpy.spin(broadcaster)

if __name__=="__main__":
    main()
