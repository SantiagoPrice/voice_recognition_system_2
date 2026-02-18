#!/usr/bin/env python3

import json
import vosk
import queue
import threading
import rclpy
import time
import numpy as np
import pyttsx3
import math
import random
import time
import re
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Empty
from rclpy.node import Node
from rclpy.parameter import Parameter as RclpyParameter
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import Parameter as MsgParameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped, PoseStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from std_msgs.msg import String
from std_msgs.msg import Int32
from openpyxl import Workbook, load_workbook
from datetime import datetime

#----------------------------------------------------
class Assessment(Node):
    def __init__(self):
        super().__init__('Assessment_node')
        self.get_logger().info("==================================================\n")
        self.goal_num = 1
        self.edge_x = 0.0
        self.edge_y = 0.0
        self.vis = 0.0
        self.command = None
        self.LLM_out = None
        self.edge_list = None
        self.out_edge_num = None
        self.prediction_requested = False
        self.state = {"table": {"pos": None, "quat": None, "size": None},}
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"prompt_test_{timestamp}.xlsx"
        header = ["Scenario", "Answer Label", "Request", "LLM Output", "Judged Label", "True/False"]
        wb = Workbook()
        ws = wb.active
        ws.append(header)
        wb.save(self.filename)

        #---- publish ----#
        self.target_point_pub = self.create_publisher(Marker, '/target_point', 10)
        self.arrow_marker = self.create_timer(1.0, self.publish_arrow_marker)    
        self.LLM_out_line_pub = self.create_publisher(Marker, '/LLM_out_line', 10)
        self.LLM_marker = self.create_timer(0.1, self.publish_LLM_out_marker)    
        self.map_update_pub = self.create_publisher(Float32MultiArray, '/map_update', 10)     
        # ---- subscribe ----
        self.sub_table = self.create_subscription(Marker,
                "/table_marker",
                lambda msg: self.marker_cb(msg, "table"),
                10
                )

        self.command_sub = self.create_subscription(
                String,
                "/command",
                self.command_cb,
                10
                )
        self.LLM_out_sub = self.create_subscription(
                String,
                "/LLM_out",
                self.LLM_out_cb,
                10
                )
        self.edge_sub = self.create_subscription(
                String,
                "/edge_list",
                self.edge_cb,
                10
                )
        self.prediction_sub = self.create_subscription(
                Empty,
                '/prediction',
                self.prediction_cb,
                10
                )
                
#=============== "CB" =================================================
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~               
    def marker_cb(self, msg: Marker, object_name: str):
        if object_name not in self.state:
            return

        p = msg.pose.position
        q = msg.pose.orientation
        s = msg.scale

        curr = (
            p.x, p.y, p.z,
            q.x, q.y, q.z, q.w,
            s.x, s.y, s.z
        )
        
        self.state[object_name]["pos"]  = (p.x, p.y, p.z)
        self.state[object_name]["quat"] = (q.x, q.y, q.z, q.w)
        self.state[object_name]["size"] = (s.x, s.y, s.z)

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def prediction_cb(self, msg):

        self.get_logger().info("prediction received")

        self.prediction_requested = True
        time.sleep(3.0)
        self.try_process()

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def command_cb(self, msg: String):
        self.command = msg.data.strip()
        if self.command == None:
            self.get_logger().info("command is None")
            return
        self.get_logger().info(f"command received: {self.command}")

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def LLM_out_cb(self, msg: String):
        self.LLM_out = msg.data.strip()
        self.get_logger().info("LLM out received")

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def edge_cb(self, msg: String):
        self.edge_list = msg.data.strip()
        #self.get_logger().info("edge list received")

#======================================================================

    def try_process(self):

        if not self.prediction_requested:
            return
        if self.LLM_out is None:
            self.get_logger().info("LLM_out not received yet")
            return
        if self.edge_list is None:
            self.get_logger().info("edge_list not received yet")
            return
        if self.command is None:
            self.get_logger().info("command not received yet")
            return

        self.get_logger().info("")
        self.get_logger().info("--------------- Result ----------------")
        # case判別
        case = self.case_judge(self.edge_list)
        self.get_logger().info(f"case: {case}")
        # command出力
        self.get_logger().info(f"Command: {self.command}")
        # LLM出力テキスト →　判別エッジラベル
        LLM_edge = self.result_edge(self.LLM_out, self.edge_list)
        self.get_logger().info(f"LLM Edge: {LLM_edge}")
        # 設定ゴール番号 → ゴールエッジラベル
        goal_edge = self.goal_edge_detect(self.goal_num, self.edge_list)
        self.get_logger().info(f"Goal Edge: {goal_edge}")
        # ゴールとLLMの結果の比較結果
        #self.get_logger().info(f"LLM_num: {self.out_edge_num}")
        #self.get_logger().info(f"goal_num: {self.goal_num}")
        T_or_F = self.out_goal_comp(self.goal_num, self.out_edge_num)
        self.get_logger().info(f"Result: {T_or_F}")
        # 結果の保存
        self.save_data(case, self.command, self.LLM_out, LLM_edge, goal_edge, T_or_F)
        self.get_logger().info("(Save result)")
        self.get_logger().info("---------------------------------------")
        # LLM_lineの表示
        self.vis = 1.0
        time.sleep(5)
        self.vis = 0.0
        self.get_logger().info("")
        
        #~~~~~~~~~~~~~~次のループ開始~~~~~~~~~~~~~~~~~~~~#
        values = Float32MultiArray()
        self.goal_num = random.randint(1, 4)
        while True:
            self.robot_x = round(random.uniform(-3, 3), 1)
            self.robot_y = round(random.uniform(-3, 3), 1)
            self.robot_yaw = round(random.uniform(0, 359), 1)
            if (abs(self.robot_x) > 1.0) and (abs(self.robot_y) > 1.0):
                break
        self.get_logger().info("------ Robot position is updated ------")
        self.get_logger().info(f"robot_x: {self.robot_x}")
        self.get_logger().info(f"robot_y: {self.robot_y}")
        self.get_logger().info("---------------------------------------\n")
        values.data = [self.robot_x, self.robot_y, self.robot_yaw, 1.0]
        self.map_update_pub.publish(values)

        # ===== リセット =====
        self.prediction_requested = False
        self.LLM_out = None

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def case_judge(self, edge_list):
        if edge_list is None:
            return None

        edges_split = self.edge_list_split(edge_list)
        
        if any("Near right edge" in e for e in edges_split):
            return "case2"
        elif any("Near edge" in e for e in edges_split):
            return "case1"
        return None

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def goal_edge_detect(self, goal_num, edge_list):
        edges_split = self.edge_list_split(self.edge_list)
        for n in range(4):
            if goal_num == (n+1):
                goal_edge = edges_split[n]
                break
        return goal_edge
                
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def edge_list_split(self, edge_list):
        edges = edge_list.splitlines()
        edges_split = [edge.split(":")[1].split("/")[0].strip() for edge in edges if edge.strip()]
        return edges_split


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def out_goal_comp(self, goal_num, out_num):
        if goal_num == out_num:
            return True
        else:
            return False

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def result_edge(self, LLM_out, edge_list):
        self.out_edge = None

        edges_split = self.edge_list_split(edge_list)

        #print(LLM_out)
        for n in range(4):
            #print(edges_split[n])
            if edges_split[n] in LLM_out:
                self.out_edge = edges_split[n]
                self.out_edge_num = n + 1
                break

        lines = edge_list.splitlines()
        match = re.search(r"\(([-\d.]+),\s*([-\d.]+)\)", lines[n])
       
        if match:
            self.edge_x = float(match.group(1))
            self.edge_y = float(match.group(2))

        if self.out_edge is None:
            self.get_logger().info("(No matching edge found in LLM output)")
            return

        if self.goal_num < 0 or self.goal_num >= len(edges_split):
            self.get_logger.info("(((gola_num out of range)))")
            return

        return self.out_edge


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def save_data(self, case, command, LLM_out, LLM_edge, goal_edge, TF):
        wb = load_workbook(self.filename)
        ws = wb.active
        ws.append([case, command, LLM_out, LLM_edge, goal_edge, TF])
        wb.save(self.filename)

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def publish_arrow_marker(self):
        # ----- arrow ------#
        now = self.get_clock().now().to_msg()
        if self.state["table"]["pos"] is None:
            return

        table_x, table_y, table_z = self.state["table"]["pos"]
        table_size_x, table_size_y, table_size_z = self.state["table"]["size"]
        table_qx, table_qy, table_qz, table_qw = self.state["table"]["quat"]

        arrow = Marker()
        arrow.header.frame_id = "map"
        arrow.header.stamp = now
        arrow.ns = "target_point"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD

        start = Point()
        end = Point()

        offset_end = 0.2
        offset_start = offset_end + 0.7

        if self.goal_num == 1:
            end.x = table_x + (table_size_x / 2) + offset_end
            end.y = table_y
            end.z = table_z
            start.x = table_x + (table_size_x / 2) + offset_start
            start.y = table_y
            start.z = table_z
        if self.goal_num == 3:
            end.x = table_x - (table_size_x / 2) - offset_end
            end.y = table_y
            end.z = table_z
            start.x = table_x - (table_size_x / 2) - offset_start
            start.y = table_y
            start.z = table_z
        if self.goal_num == 4:
            end.x = table_x
            end.y = table_y + (table_size_y / 2) + offset_end
            end.z = table_z
            start.x = table_x
            start.y = table_y + (table_size_y / 2) + offset_start
            start.z = table_z
        if self.goal_num == 2:
            end.x = table_x
            end.y = table_y - (table_size_y / 2) - offset_end
            end.z = table_z
            start.x = table_x
            start.y = table_y - (table_size_y / 2) - offset_start
            start.z = table_z

        arrow.points = [start, end]

        # 矢印の太さ
        arrow.scale.x = 0.075  # シャフト径
        arrow.scale.y = 0.21   # ヘッド径
        arrow.scale.z = 0.21   # ヘッド長
        arrow.color.r = 1.0
        arrow.color.g = 0.0
        arrow.color.b = 0.0
        arrow.color.a = 1.0

        self.target_point_pub.publish(arrow)
        # ------------------#

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def publish_LLM_out_marker(self):

        if self.state["table"]["pos"] is None:
            return

        now = self.get_clock().now().to_msg()

        table_x, table_y, table_z = self.state["table"]["pos"]
        table_size_x, table_size_y, table_size_z = self.state["table"]["size"]

        line = Marker()
        line.header.frame_id = "map"
        line.header.stamp = now
        line.ns = "LLM_outline"
        line.id = 2
        line.type = Marker.LINE_LIST
        line.action = Marker.ADD

        line.scale.x = 0.5

        line.color.r = 0.0
        line.color.g = 0.0
        line.color.b = 1.0
        line.color.a = self.vis

        start = Point()
        end = Point()
        if self.edge_x == 0.0:
            start.x = self.edge_x + (table_size_x/2)
            start.y = self.edge_y
            start.z = table_z + (table_size_z/2)
            end.x = self.edge_x - (table_size_x/2)
            end.y = self.edge_y
            end.z = table_z + (table_size_z/2)
        if self.edge_y == 0.0:
            start.x = self.edge_x
            start.y = self.edge_y + (table_size_y/2)
            start.z = table_z + (table_size_z/2)
            end.x = self.edge_x
            end.y = self.edge_y - (table_size_y/2)
            end.z = table_z + (table_size_z/2)

        #line.points = [start, end]
        line.points.append(start)
        line.points.append(end)
        line.lifetime = Duration(sec=0, nanosec=0)

        self.LLM_out_line_pub.publish(line)



def main():
    rclpy.init()
    assessment = Assessment()
    rclpy.spin(assessment)

if __name__=="__main__":
    main()
