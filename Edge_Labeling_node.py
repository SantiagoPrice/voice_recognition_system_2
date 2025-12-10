import sys
import rclpy
import requests
import time
import numpy as np
import math
import random
from rclpy.node import Node
from rclpy.parameter import Parameter as RclpyParameter
from rcl_interfaces.msg import Parameter as MsgParameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped, PoseStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from std_msgs.msg import String
from msgs.msg import Data

class EdgeLabeling(Node):
    def __init__(self):
        super().__init__("Edge_Labeling")

        self.edge_list = self.create_publisher(String, "/edge_list", 10)

        self.sub = self.create_subscription(
                Data,
                "/target_marker",
                self.PoseSelection,
                10
                )

    def PoseSelection(self, target):
        #--------------------------------------------------------
        robot_name = target.rn
        robot_x = target.rx
        robot_y = target.ry
        robot_yaw_deg = target.rd
        robot_yaw_rad = math.radians(robot_yaw_rad)
        robot_size_x = target.rsx
        robot_size_y = target.rsy
        robot_size_z = target.rsz

        table_name = target.tn
        table_x = target.tx
        table_y = target.ty
        table_yaw_deg = target.td
        table_yaw_rad = math.radians(table_yaw_deg)
        table_size_x = target.tsx
        table_size_y = target.tsy
        table_size_z = target.tsz

        THETA_A = target.tha

        table_hsize_x = table_size_x / 2
        table_hsize_y = table_size_y / 2

        corners = [
                (((table_x+table_hsize_x)*math.cos(table_yaw_deg)), ((table_y+table_hsize_y)*(-sin(table_yaw_deg))), table_size_z),
                (((table_x-table_hsize_x)*math.cos(table_yaw_deg)), ((table_y+table_hsize_y)*(-sin(table_yaw_deg))), table_size_z),
                (((table_x-table_hsize_x)*math.cos(table_yaw_deg)), ((table_y-table_hsize_y)*(-sin(table_yaw_deg))), table_size_z),
                (((table_x+table_hsize_x)*math.cos(table_yaw_deg)), ((table_y-table_hsize_y)*(-sin(table_yaw_deg))), table_size_z)
                ]
        corner_ijkl = np.zeros((4,5))

        self.get_logger().info("Recived data")
        #----------------------------------------------------------
        

        #distance between robot and table
        Pc = np.array([robot_x - table_x, robot_y - table_y])
        pc_dis = np.linalg.norm(Pc)
        angle_pc = np.degrees(np.arctan2(Pc[0], Pc[1]))
        
        #distance between center of table and each corner
        #calculate theta for each corner
        #corner_A
        pA_x = table_x + (table_size_x / 2) - table_x 
        pA_y = table_y + (table_size_y / 2) - table_y
        PA = np.array([pA_x, pA_y])
        pA_dis = np.linalg.norm(PA)
        angle_pA = np.degrees(np.arctan2(PA[0], PA[1]))
        theta_A = (angle_pc - angle_pA + 360) % 360
        #corner_B
        pB_x = table_x - (table_size_x / 2) - table_x
        pB_y = table_y + (table_size_y / 2) - table_y
        PB = np.array([pB_x, pB_y])
        pB_dis = np.linalg.norm(PB)
        angle_pB = np.degrees(np.arctan2(PB[0], PB[1]))
        theta_B = (angle_pc - angle_pB + 360) % 360
        #corner_C
        pC_x = table_x - (table_size_x / 2) - table_x
        pC_y = table_y - (table_size_y / 2) - table_y
        PC = np.array([pC_x, pC_y])
        pC_dis = np.linalg.norm(PC)
        angle_pC = np.degrees(np.arctan2(PC[0], PC[1]))
        theta_C = (angle_pc - angle_pC + 360) % 360
        #corner_D
        pD_x = table_x + (table_size_x / 2) - table_x
        pD_y = table_y - (table_size_y / 2) - table_y
        PD = np.array([pD_x, pD_y])
        pD_dis = np.linalg.norm(PD)
        angle_pD = np.degrees(np.arctan2(PD[0], PD[1]))
        theta_D = (angle_pc - angle_pD + 360) % 360

        #identify corners
        id_theta = [[1,theta_A], [2,theta_B], [3,theta_C], [4,theta_D]]
        sorted_theta = sorted(id_theta, key=lambda x: x[1])
        corner_or = id_theta
        print(id_theta)
        for n in range(4):
            if (sorted_theta[n][0] == 1):
                corner_or[n] = np.array([pA_dis, theta_A, pA_x+table_x, pA_y+table_y, table_size_z])
            elif (sorted_theta[n][0] == 2):
                corner_or[n] = np.array([pB_dis, theta_B, pB_x+table_x, pB_y+table_y, table_size_z])
            elif (sorted_theta[n][0] == 3):
                corner_or[n] = np.array([pC_dis, theta_C, pC_x+table_x, pC_y+table_y, table_size_z])
            elif (sorted_theta[n][0] == 4):
                corner_or[n] = np.array([pD_dis, theta_D, pD_x+table_x, pD_y+table_y, table_size_z])
        corner_i = np.array([corner_or[0][0], corner_or[0][1], corner_or[0][2], corner_or[0][3], corner_or[0][4]])
        corner_j = np.array([corner_or[1][0], corner_or[1][1], corner_or[1][2], corner_or[1][3], corner_or[1][4]])
        corner_k = np.array([corner_or[2][0], corner_or[2][1], corner_or[2][2], corner_or[2][3], corner_or[2][4]])
        corner_l = np.array([corner_or[3][0], corner_or[3][1], corner_or[3][2], corner_or[3][3], corner_or[3][4]])
        
        corner_ijkl
        corner_ijkl = np.array([corner_i, corner_j, corner_k, corner_l])
        print(corner_ijkl)

        #modality identification
        for _ in range(1):
            if corner_ijkl[0][1] < THETA_A:
                edge_1 = "Near left edge"
                edge_2 = "Near right edge"
                edge_3 = "Far right edge"
                edge_4 = "Far left edge"
                continue
            elif corner_ijkl[3][1] < (360 - THETA_A):
                edge_1 = "Near edge"
                edge_2 = "Right edge"
                edge_3 = "Far edge"
                edge_4 = "Left edge"
                continue
            edge_1 = "Near right edge"
            edge_2 = "Far right edge"
            edge_3 = "Far left edge"
            edge_4 = "Near left edge"
        
        print(edge_1, edge_2, edge_3, edge_4, sep="\n")
        
        edge = String()
        edge.no1 = edge_1
        edge.no2 = edge_2
        edge.no3 = edge_3
        edge.no4 = edge_4

        self.edge_list.publish(edge)


def main():
    rclpy.init()
    edgelabeling = EdgeLabeling()

if __name__=="__main__":
    main()


