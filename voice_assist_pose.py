import sys
import os
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

GpuInit()

#--------------------------------------------------
MODEL_PATH = "/home/orin/voice_recognition_system/vosk-model-lm/vosk-model-en-us-0.22"
model = Model(MODEL_PATH)

API_KEY = ' '
API_URL = "https://openrouter.ai/api/v1/chat/completions"

engine = pyttsx3.init()
engine.setProperty("rate", 150)
engine.setProperty("volume", 1.0)

headers = {
        "Authorization": f"Bearer {API_KEY}",
        "Content-Type": "application/json"
        }

q = queue.Queue()
#---------------------------------------------------
robot_x = 1.5
robot_y = 4.5
robot_yaw_deg = 270.0
robot_yaw_rad = math.radians(robot_yaw_deg)
robot_name = "robot"
Robot = np.array([robot_x, robot_y, robot_yaw_deg, robot_yaw_rad, robot_name]) 
robot_size_x = 0.5
robot_size_y = 0.5
robot_size_z = 1.0

table_x = 2.0
table_y = 3.0
table_yaw_deg = 0.0
table_yaw_rad = math.radians(table_yaw_deg)
table_name = "table"
Table = np.array([table_x, table_y, table_yaw_deg, table_yaw_rad, table_name]) 
table_size_x = 1.5
table_size_y = 0.9
table_size_z = 0.75

THETA_A = 15.0

table_hsize_x = table_size_x / 2
table_hsize_y = table_size_y / 2
corners = [
        ((table_x + table_hsize_x), (table_y + table_hsize_y), table_size_z),
        ((table_x - table_hsize_x), (table_y + table_hsize_y), table_size_z),
        ((table_x - table_hsize_x), (table_y - table_hsize_y), table_size_z),
        ((table_x + table_hsize_x), (table_y - table_hsize_y), table_size_z)
        ]
corner_ijkl = np.zeros((4,5))
#----------------------------------------------------
edge_flag = 0
req_edge = 0
target = "None"

############################################################################
def audio_callback(indata, frames, time, status):
    if status:
        print(status, file = sys.stderr)
    q.put(bytes(indata))

###########################################################################
class CreateNode(Node):
    def __init__(self):
        super().__init__("Creater")
        print(f"Started {target}")
        print("this is CreateNode")
        self.subsctiption = self.create_subscription(
                String,
                "target",
                self.publish_node,
                10
                )

    def publish_node(self, msg):
        print("start CreateNode")
        self.goal = msg.data
        
        self.cli_tr = self.create_client(SetParameters, '/p1_trig_vint_node/set_parameters')
        self.cli_par = self.create_client(SetParameters, '/tracker_with_cloud_node/set_parameters')

        """
        while not self.cli_par.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /p1_trig_vint_node/set_parameters service...')

        while not self.cli_tr.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /tracker_with_cloud_node/set_parameters service...')
        """

        param = MsgParameter()
        param.name = 'GOAL'
        param.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=self.goal)
        req = SetParameters.Request()
        req.parameters = [param]
        self.future = self.cli_par.call_async(req)

        param.name = 'goal_trigg'
        param.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True)
        self.future = self.cli_tr.call_async(req)

        self.get_logger().info(f"Declared Parameter 'GOAL' with value: {self.goal}")

###########################################################################3
def launch_ros_node(node_class, spin=False):
    def target():
        node = node_class()
        if spin:
            rclpy.spin(node)
            print("spining")
        else:
            rclpy.spin_once(node, timeout_sec=1.0)
    threading.Thread(target=target, daemon=True).start()

#############################################################################
def spin_rviz(node):
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

############################################################################

# voice_recognition
class VoiceRecognition(Node):
    def __init__(self):
        super().__init__("voice")
        self.wake_word = "assistant"
        self.listening_for_command = False
        self.stop = False
        self.flag = 0
        print("===================================================")
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def run(self, edges):
        samplerate = int(sd.query_devices(None, "input")["default_samplerate"])
        print(f"actual sample rate is {samplerate}")
        with sd.RawInputStream(samplerate=samplerate, blocksize=int(samplerate*1), dtype="int16",
                               channels=1, callback=audio_callback, device=None):
            rec = vosk.KaldiRecognizer(model, samplerate)
            print("Listening for wake word...")
            self.read=False

            while True:
                if self.stop == True:
                    continue
                data = q.get()
                audio_np = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
                volume =  max(np.abs(audio_np))
           
                # threshold_of_volume
                if volume < 0.1:
                    if self.listening_for_command  == True:
                        print("Listening...")
                    else:
                        print("Listening for wake word...")
                else:
                    self.read=True
                    print(f"Volume: {volume:.4f}")
                
                if not self.read:
                    continue
                
                if rec.AcceptWaveform(data):
                    self.read=False
                    result = json.loads(rec.Result())
                    #text = result.get("text", "").strip().lower()
                    text = input("test sentence: ")
                    print(f"audio recording: {text}")
                    if not text == "assistant":
                        text += "_pose/selection"
                    if text:
                        if text.split("_")[-1] == "pose/selection":
                            #output = self.LLM_edge_case1(text.split("_")[0], edges)
                            output = input("edge:")
                            global table_x
                            global table_y 
                            global table_yaw_rad
                            table_x = random.uniform(2,15)
                            table_y = random.uniform(2,15)
                            #table_yaw_rad = math.radians(random.uniform(0,360))
                            global corners
                            corners = [
                                    ((table_x + table_hsize_x), (table_y + table_hsize_y), table_size_z),
                                    ((table_x - table_hsize_x), (table_y + table_hsize_y), table_size_z),
                                    ((table_x - table_hsize_x), (table_y - table_hsize_y), table_size_z),
                                    ((table_x + table_hsize_x), (table_y - table_hsize_y), table_size_z)
                                    ]

                            self.voice_feedback(output)
                            global edge_flag
                            edge_flag = 1
                            self.result_edge(output, edges)
                        print(f"(Detected: {text})")
                        if not self.listening_for_command and self.flag != 1:
                            if self.wake_word in text:
                                #self.LLM(text)
                                self.response(text)
                                self.listening_for_command = True
                        elif edge_flag == 0:
                            self.stop = True
                            output = self.LLM(text)
                            self.voice_feedback(output)
                            self.process_command(output)
                            self.listening_for_command = False
                            if self.flag == 1:
                                print("Listening...")
                else:
                    partial_result = json.loads(rec.PartialResult())
                    print(f"Partial: {partial_result.get('partial', '')}")

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def response(self, text):
        if text == "assistant":
            print("Hello, How can I help you.")
            engine.say("Hello, How can I help you.")
            engine.runAndWait()
        else:
            print("Listening...")
            partial_result = json.loads(rec.Result())
            print(f"Partial: {partial_result.get('partial', '')}")

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def voice_feedback(self, text):
        engine.say(text)
        engine.runAndWait()

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def LLM(self, text):
        base_prompt = """
        You are a friendly AI assistant built into an electric wheelchair used in a bedroom. 
        The room has destinations: 
        - bed
        - table. 
        commands
        - stop
        When the user speaks, always respond in one short, natural English sentence that clearly includes the destination or commands
        Do not ask questions. Instead, acknowledge the user’s state in a warm way and declare the action to move to the destination or commands. 
        The response should sound natural when read aloud by a speaker. 
        If you get the word "assistant" from the User, you have to response "Hello, How can I help you."
        Examples: 
        User: “I’m hungry.” 
        Assistant: “Alright, let’s move to the table so you can eat.” 
        User: “I’m sleepy.” 
        Assistant: “Got it, heading to the bed now.”

        User: 
        """
        prompt = base_prompt + text
        print(prompt)
        data = {
                "model": "google/gemma-3n-e4b-it:free",
                "messages": [{"role": "user", "content": prompt}]
                }
        response = requests.post(API_URL, json=data, headers=headers)
        print(response)
        result = response.json()
        output = result['choices'][0]['message']['content']
        print("Assistant:", output)
        return output

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def LLM_edge_case1(self, text, edges):
        base_prompt = """You are an assistant AI mounted on a wheelchair.
        You are currently near a table, and there are four possible positions around it:
        "Near Edge"
        "Far Edge"
        "Right Edge"
        "Left Edge"
        Near edge is the edge which is most close to wheelchair. 
        When the user makes a request, determine the most suitable position based on their intent. 
        Respond in a polite and natural tone as an assistant AI. 
        Your response must include the label of the selected position (e.g., “Far edge”). 
        Please response with just one or two sentences. 

        User request:
        """
        prompt = base_prompt + text
        #print(prompt)
        data = {
                "model": "google/gemma-3n-e4b-it:free",
                "messages": [{"role": "user", "content": prompt}]
                }
        response = requests.post(API_URL, json=data, headers=headers)
        #print(response)
        result = response.json()
        output = result["choices"][0]["message"]["content"]
        print("Assistant:", output)
        return output

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def LLM_edge_case2(self, text, edges):
        base_prompt_1 = """You are an assistant AI mounted on a wheelchair.\nYou are currently near a table, and there are four possible positions around it:"""
        base_prompt_2 = """Near edge is the edge which is most close to wheelchair. When the user makes a request, determine the most suitable position based on their intent. Respond in a polite and natural tone as an assistant AI. Your response must include the label of the selected position (e.g., “Far edge”). Please response with just one or two sentences. \nUser request:
        """
        prompt = base_prompt_1 + "\n" + edges[0] + "\n" + edges[1] + "\n" + edges[2] + "\n" + edges[3] + "\n" + base_prompt_2 + text
        #print(prompt)
        data = {
                "model": "google/gemma-3n-e4b-it:free",
                "messages": [{"role": "user", "content": prompt}]
                }
        response = requests.post(API_URL, json=data, headers=headers)
        #print(response)
        result = response.json()
        output = result["choices"][0]["message"]["content"]
        print("Assistant:", output)
        return output

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def result_edge(self, output, edges):
        edge_number = 0
        for m in range(4):
            if edges[m] in output:
                edge_number = m
                continue
        print(edge_number)
        global req_edge
        req_edge = edge_number

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def process_command(self, output):
        self.publisher = self.create_publisher(String, "target", 10)
        msg = String()
        if "bed" in output:
            msg.data = "59"
            self.publisher.publish(msg)
            goal = "59"
            self.flag = 0
        elif "table" in output:
            msg.data = "60"
            self.publisher.publish(msg)
            goal = "60"
            self.flag = 0
        elif "stop" in output:
            target = "stop_node"
            goal = "1"
            self.flag = 0
        elif "again" in output:
            print("Please say again")
            self.flag = 1
        elif "end" in output:
            self.flag = 0
        else:
            self.flag = 2
        
        self.stop = False
        
#######################################################################
class MapBroadcaster(Node):
    def __init__(self):
        super().__init__('map_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.table_marker_pub = self.create_publisher(Marker, '/table_marker', 10)
        self.robot_marker_pub = self.create_publisher(Marker, '/robot_marker', 10)
        self.RtoT_marker_pub = self.create_publisher(Marker, '/RtoT_marker', 10)
        self.Tcon_marker_pub = self.create_publisher(Marker, '/Tcon_marker', 10)
        self.Redge_marker_pub = self.create_publisher(Marker, '/Redge_marker', 10)
        self.robot_pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1Hz
        self.vis_edge = 0.0
        
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def yaw_to_quaternion(self, yaw):
        half = yaw * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def timer_callback(self):
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
        m_table.pose.position.z = table_height / 2.0
        m_table.pose.orientation.x = q_table[0]
        m_table.pose.orientation.y = q_table[1]
        m_table.pose.orientation.z = q_table[2]
        m_table.pose.orientation.w = q_table[3]
        # table_size
        m_table.scale.x = table_size_x
        m_table.scale.y = table_size_y
        m_table.scale.z = table_height
        m_table.color.r = 0.0
        m_table.color.g = 0.0
        m_table.color.b = 1.0
        m_table.color.a = 0.5
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

        if edge_flag == 1:
            self.vis_edge = 1.0
        m_Redge = Marker()
        m_Redge.header.stamp = now
        m_Redge.header.frame_id = "map"
        m_Redge.ns = "line"
        m_Redge.id = 3
        m_Redge.type = Marker.LINE_LIST
        m_Redge.action = Marker.ADD
        m_Redge.scale.x = 0.04
        m_Redge.color.r = self.vis_edge
        m_Redge.color.g = 0.0
        m_Redge.color.b = 0.0
        m_Redge.color.a = self.vis_edge
        start_point_Redge = Point()
        start_point_Redge.x = corner_ijkl[req_edge][2]
        start_point_Redge.y = corner_ijkl[req_edge][3]
        start_point_Redge.z = corner_ijkl[req_edge][4]
        end_point_Redge = Point()
        end_point_Redge.x = corner_ijkl[(req_edge+3)%4][2]
        end_point_Redge.y = corner_ijkl[(req_edge+3)%4][3]
        end_point_Redge.z = corner_ijkl[(req_edge+3)%4][4]
        m_Redge.points.append(start_point_Redge)
        m_Redge.points.append(end_point_Redge)
        m_Redge.lifetime = Duration(sec=0, nanosec=0)
        self.Redge_marker_pub.publish(m_Redge)

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #------pose selection------------
    def PoseSelection(self):
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
        
        global corner_ijkl
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
        edges = np.array([edge_1, edge_2, edge_3, edge_4])
        return(edges)

##############################################################
def main():
    rclpy.init()

    broadcaster = MapBroadcaster()
    voice = VoiceRecognition()
    map_pose = broadcaster.PoseSelection()

    executor = MultiThreadedExecutor()
    executor.add_node(broadcaster)
    executor.add_node(CreateNode())
    spin_thread = threading.Thread(target=executor.spin, daemon=True)

    spin_thread.start()
    voice.run(map_pose)
    """
    except KeyboradInterrput:
        pass
    finally:
        broadcaster.destroy_node()
        voice.destroy_node()
        creater.destroy_node()
        rclpy.shutdown()
    """

##############################################################
if __name__ == "__main__":
    main()
