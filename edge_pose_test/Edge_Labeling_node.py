#!/usr/bin/env python3
import rclpy
import numpy as np
import math
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import String


class EdgeLabeling(Node):
    def __init__(self):
        super().__init__("Edge_Labeling")

        self.edge_pub = self.create_publisher(String, "/edge_list", 10)

        self.state = {
            "robot": {"pos": None, "quat": None, "size": None},
            "table": {"pos": None, "quat": None, "size": None},
        }

        self.updated = {"robot": False, "table": False}
        self.prev_state = {"robot": None, "table": None}

        self.latest_edge_text = None
        self.edge_ready = False

        # ===== 絶対エッジ番号（テーブル中心基準）=====
        self.fixed_edges = {
            "edge_1": np.array([0.75, 0.0]),
            "edge_2": np.array([0.0, 0.45]),
            "edge_3": np.array([-0.75, 0.0]),
            "edge_4": np.array([0.0, -0.45]),
        }

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

    # ------------------------------------------------
    def PoseSelection(self):

        # ===== 状態取得 =====
        robot_x, robot_y, _ = self.state["robot"]["pos"]
        table_x, table_y, _ = self.state["table"]["pos"]
        sx, sy, _ = self.state["table"]["size"]
        tqx, tqy, tqz, tqw = self.state["table"]["quat"]

        yaw = self.quat_to_yaw(tqx, tqy, tqz, tqw)

        hx, hy = sx / 2.0, sy / 2.0
        c, s = math.cos(yaw), math.sin(yaw)

        # ===== エッジ中心（local）=====
        local_edges = {
            "E0": np.array([+hx, 0.0]),
            "E1": np.array([0.0, +hy]),
            "E2": np.array([-hx, 0.0]),
            "E3": np.array([0.0, -hy]),
        }

        local_corners = {
                "C0":  np.array([+hx, -hy]),
                "C1":  np.array([+hx, +hy]),
                "C2":  np.array([-hx, +hy]),
                "C3":  np.array([-hx, -hy]),
                }

        world_edges = {}
        local_rotated = {}
        local_rotated_corner = {}
        local_rotated_edge = {}
        corner_theta = {}
        edge_theta = {}

        # ===== ロボット→中心ベクトル =====
        Pc = np.array([robot_x - table_x, robot_y - table_y])
        angle_TR = np.degrees(np.arctan2(Pc[0], Pc[1]))

        for name, (lx, ly) in local_edges.items():
            ex = lx * c - ly * s
            ey = lx * s - ly * c

            local_rotated_edge[name] = np.array([ex, ey])
        
        for name, (lx, ly) in local_corners.items():
            cx = lx * c - ly * s
            cy = lx * s - ly * c

            local_rotated_corner[name] = np.array([cx, cy])

            angle_corner = np.degrees(np.arctan2(cx, cy))
            theta = (angle_TR - angle_corner + 360.0) % 360.0
            corner_theta[name] = theta


        # ===== θでソート（右回り）=====
        sorted_corner = sorted(corner_theta.items(), key=lambda x: x[1])
        order = [e[0] for e in sorted_corner]
        # エッジの名前順番入れ替え
        original_corner_names = list(local_corners.keys())
        sort_idx = [original_corner_names.index(name) for name in order]
        edge_names = list(local_edges.keys())
        edge_order = [edge_names[i] for i in sort_idx]

        # ===== semanticラベル =====
        THETA_A = 10.0

        if sorted_corner[0][1] < THETA_A:
            labels = ["Near left edge",
                      "Near right edge",
                      "Far right edge",
                      "Far left edge"]
        elif sorted_corner[3][1] < (360.0 - THETA_A):
            labels = ["Near edge",
                    "Right edge",
                    "Far edge",
                    "Left edge"]
        else:
            labels = ["Near right edge",
                      "Far right edge",
                      "Far left edge",
                      "Near left edge"]

        semantic_map = {}
        for phys, label in zip(order, labels):
            semantic_map[phys] = label

        # =================================================
        # 循環割当（修正版）
        # =================================================

        ref_phys = edge_order[0]

        min_d = float("inf")
        ref_edge_id = None
        comp_edge = {} 

        # ★ 修正：local_rotated で比較
        for edge_id, fixed_pos in self.fixed_edges.items():
            d = np.linalg.norm(local_rotated_edge[ref_phys] - fixed_pos)
            if d < min_d:
                min_d = d
                ref_edge_id = edge_id

        edge_ids = ["edge_1", "edge_2", "edge_3", "edge_4"]
        ref_index = edge_ids.index(ref_edge_id)

        assignment = {}

        for i in range(4):
            edge_id = edge_ids[(ref_index + i) % 4]
            assignment[edge_id] = order[i]

        # ===== 出力 =====
        lines = []

        self.get_logger().info("-----------------------------------")
        for edge_id in edge_ids:
            phys = assignment[edge_id]
            sem = semantic_map[phys]
            x, y = self.fixed_edges[edge_id]

            line = f"{edge_id}: {sem} / ({x:.2f}, {y:.2f})"
            lines.append(line)
            self.get_logger().info(line)

        self.latest_edge_text = "\n".join(lines)
        self.edge_ready = True

    # ------------------------------------------------
    def publish_edge(self):
        if self.edge_ready:
            msg = String()
            msg.data = self.latest_edge_text
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

