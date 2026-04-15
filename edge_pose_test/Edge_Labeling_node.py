#!/usr/bin/env python3

import rclpy
import numpy as np
import math
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import String
import json

class EdgeLabeling(Node):
    def __init__(self):
        super().__init__("Edge_Labeling")

        self.edge_pub = self.create_publisher(String, "/edge_list", 10)
        self.cposes_pub = self.create_publisher(Marker, "/cand_poses", 10)

        self.state = {
            "robot": {"pos": None, "quat": None, "size": None},
            "table": {"pos": None, "quat": None, "size": None},
        }

        self.updated = {"robot": False, "table": False}
        self.prev_state = {"robot": None, "table": None}

        self.latest_edge_text = None
        self.edge_ready = False

        self.encripted_elist = None

        # ===== 絶対エッジ番号（テーブル中心基準）=====

        self.sub_robot = self.create_subscription(
            Marker, "/robot_marker",
            lambda msg: self.cb_marker(msg, "robot"), 10)

        self.sub_table = self.create_subscription(
            Marker, "/table_marker",
            lambda msg: self.cb_marker(msg, "table"), 10)

        self.pub_timer = self.create_timer(1.0, self.publish_edge)

    # ------------------------------------------------
    def is_changed(self, prev, curr, eps=1e-3):
        if prev is None:
            return True
        return any(abs(p - c) > eps for p, c in zip(prev, curr))

    def cb_marker(self, msg: Marker, name: str):
        p, q, s = msg.pose.position, msg.pose.orientation, msg.scale
        curr = (p.x, p.y, p.z, q.x, q.y, q.z, q.w, s.x, s.y, s.z)

        if self.is_changed(self.prev_state[name], curr):
            self.prev_state[name] = curr
            self.updated[name] = True

            self.state[name]["pos"] = (p.x, p.y, p.z)
            self.state[name]["quat"] = (q.x, q.y, q.z, q.w)
            self.state[name]["size"] = (s.x, s.y, s.z)

            if self.updated["robot"] or self.updated["table"]:
                self.calc_once()
                self.updated["robot"] = False
                self.updated["table"] = False

    # ------------------------------------------------
    def quat_to_yaw(self, qx, qy, qz, qw):
        siny_cosp = 2.0 * (qw*qz + qx*qy)
        cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def have_all(self):
        for obj in ["robot", "table"]:
            for k in ["pos", "quat", "size"]:
                if self.state[obj][k] is None:
                    return False
        return True

    def calc_once(self):
        if self.have_all():
            self.PoseSelection()


    def yaw_to_quaternion(eslf, yaw: float) -> list:
        """Convert yaw angle (rad) to quaternion."""
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        q=[qx,qy,qz,qw]
        return q


    def create_arrow_marker(self, name, poses, yaw, frame_id="map"):
        marker = Marker()

        marker.header.frame_id = frame_id
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0

        marker.ns = "rso2_arrows"
        marker.id = hash(name) % 10000

        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # ✅ position from poses[name]
        marker.pose.position.x = poses[0]
        marker.pose.position.y = poses[1]
        marker.pose.position.z = 0.0

        # ✅ orientation from yaw
        
        q=self.yaw_to_quaternion(yaw)
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1] 
        marker.pose.orientation.z = q[2] 
        marker.pose.orientation.w = q[3] 

        # Arrow size
        marker.scale.x = 0.2   # length
        marker.scale.y = 0.05   # width
        marker.scale.z = 0.05   # height

        # Color (RGBA)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5

        marker.lifetime.sec = 0  # forever

        self.cposes_pub.publish(marker)
    



    # ------------------------------------------------
    def PoseSelection(self):

        # ===== 状態取得 =====
        robot_x, robot_y, _ = self.state["robot"]["pos"]
        table_x, table_y, _ = self.state["table"]["pos"]
        sx, sy, _ = self.state["table"]["size"]
        tqx, tqy, tqz, tqw = self.state["table"]["quat"]

        yaw = self.quat_to_yaw(tqx, tqy, tqz, tqw)

        self.get_logger().info(f"Tilted angle {math.degrees(yaw)}")

        hx, hy = sx / 2.0, sy / 2.0
        c, s = math.cos(yaw), math.sin(yaw)

        # ===== エッジ中心（local）=====

        local_corners = [
                np.array([+hx, -hy]),
                np.array([+hx, +hy]),
                np.array([-hx, +hy]),
                np.array([-hx, -hy]),
        ]

        world_edges = {}
        local_rotated = {}
        local_rotated_corner = {}
        local_rotated_edge = []
        corner_theta = []
        edge_theta = {}
        poses = []
        global_corners=[]

        # ===== ロボット→中心ベクトル =====
        Pc = np.array([robot_x  - table_x , robot_y - table_y])
        angle_TR = np.degrees(np.arctan2(Pc[1], Pc[0]))


        
        for i, (lx, ly) in enumerate(local_corners):
            cx = lx * c - ly * s
            cy = lx * s + ly * c

            global_corners.append( np.array([cx, cy]) +  np.array([table_x, table_y]) )
            angle_corner = np.degrees(np.arctan2(cy, cx))
            theta = (angle_corner - angle_TR + 360.0) % 360.0
            corner_theta.append(theta)
            
        for i,gc in enumerate(global_corners):
            i_before= (i+3)%4
            edge= global_corners[i]-global_corners[i_before]
            edge_l= np.linalg.norm(edge)
            edge_n = np.array([edge[1],-edge[0]])/ edge_l

            pose_loc = gc - edge * 0.5 + 0.5*edge_n

            pose_loc_c = pose_loc - np.array([table_x, table_y])

            print(f"Pose locaton: {pose_loc}")
        
            pose_yaw = math.atan2( -pose_loc_c[1], -pose_loc_c[0])
            
            pose_mark = self.create_arrow_marker(f"C{i}", pose_loc , pose_yaw)
            poses.append([pose_loc , pose_yaw])


        # ===== θでソート（右回り）=====
        sorted_theta = sorted(corner_theta, key=lambda x: x)
        sorted_theta_ind = [corner_theta.index(t) for t in sorted_theta]
        sorted_poses= [poses[i] for i in sorted_theta_ind]


        # ===== semanticラベル =====
        THETA_A = 10.0

        GAMMA = (sorted_theta[0] + (360.0 - sorted_theta[3])) / 2  

        if sorted_theta[0] < THETA_A:                       # Right Reference corner INSIDE the cone
            labels = ["Near left edge / Near edge",
                      "Near right edge / Right edge",
                      "Far right edge / Far edge",
                      "Far left edge / Left edge"]

        elif sorted_theta[3] < (360.0 - THETA_A):           # NO corner INSIDE the cone, line connecting corner and center passes at the RIGHT of the wheelchair
            if sorted_theta[0] < GAMMA:                            # Simmetry axis of the near edge passes at the LEFT of the wheelchair 
                labels = ["Near edge / Near Left edge",
                        "Right edge / Near Right edge",
                        "Far edge / Far Right edge",
                        "Left edge / Far Left edge"]
            elif sorted_theta[0] > GAMMA:                          # Simmetry axis of the near edge passes at the LEFT of the wheelchair center
                labels = ["Near edge / Near Right edge",
                        "Right edge / Far Right edge",
                        "Far edge / Far Left edge",
                        "Left edge / Near Left edge"]
            else:                                                  # Simmetry axis of the near edge crosses the wheelchair center
                labels = ["Near edge / Near edge",
                        "Right edge / Right edge",
                        "Far edge / Far edge",
                        "Left edge / Left edge"]
        else:
            labels = ["Near right edge / Near edge",       # Left Reference corner INSIDE the cone
                      "Far right edge / Right edge",
                      "Far left edge / Far edge",
                      "Near left edge / Left edge"]

        semantic_map = {}


        self.lab_poses = dict (zip(labels,sorted_poses))
        

        buffer=""
        self.get_logger().info("-----------------------------------")
        for label, pose_data in zip(labels,sorted_poses):
            X = pose_data[0][0]
            Y = pose_data[0][1]
            YAW = pose_data[1]
            line = f"{label}//({X:.2f},{Y:.2f},{YAW:.2f})\n"
            self.get_logger().info(line)
            buffer+=line

        self.encripted_elist = buffer
        

        
    def publish_edge(self):    
        msg = String()
        if self.encripted_elist:
            msg.data = self.encripted_elist
            self.edge_pub.publish(msg)


def main():
    rclpy.init()
    node = EdgeLabeling()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

