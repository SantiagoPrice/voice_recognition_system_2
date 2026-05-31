#!/usr/bin/env python3

import os
import rclpy
import requests
from google import genai
import math
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from std_msgs.msg import Empty
import ast
import re
import yaml
from ament_index_python.packages import get_package_share_directory
from custom_msgs.action import VoiceConfirmation
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
import subprocess


pkg_path = get_package_share_directory('vint_ros')
API_PATH = os.path.join(pkg_path,'conf/conf.yaml')
PROMPT_PATH = os.path.join(pkg_path,'others/prompts.yaml')
AUDIO_PATH = os.path.join(pkg_path,'others/audio_samples/eng/')

with open(API_PATH, "r") as f:
    conf_handlr= yaml.safe_load(f)
    API_KEY = conf_handlr["key"]
    API_KEY_OR = conf_handlr["key_or"]


with open(PROMPT_PATH, "r") as f:
    prompt_handlr= yaml.safe_load(f)


class TaskManager(Node):
    def __init__(self):
        super().__init__("task_manager")
        self.goal_num = None
        self.goal_received_once = False
        self.edges = None 
        self.edge_last = None
        self.edge_received_once = False
        self.cmd_last = None
        self.cmd_received_once = False
        self.prompts = prompt_handlr
        self.is_LLM_enabled = True
        self.text= ""
        self._lock=threading.Lock()

        #---- VoiceConfirmation action in 2nd thread ----
        self._cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(self,
                                           VoiceConfirmation, 
                                           'voice_confirmation', 
                                           self.execute_callback,
                                           callback_group=self._cb_group)

        # ---- subscriptions ----
        self.sub_edge = self.create_subscription(String, "/edge_list", self.cb_edge, 10)
        self.sub_cmd = self.create_subscription(String, "/command", self.cb_command, 10)
         
        # ---- publish ---- #
        self.prediction = self.create_publisher(Empty, "/prediction", 10)
        self.LLM_out = self.create_publisher(String, "/LLM_out", 10)
        self.cposes_pub = self.create_publisher(Marker, "/select_poses", 10)

        if API_KEY:
            self.source = "Genai"
        elif API_KEY_OR:
            self.source="Open Router"
        else:
            raise ValueError("NO API key")
        
        self.get_logger().info(f"LLM connected via {self.source}")
           
    def execute_callback(self, goal_handle):
        self.timeout_flag = False
        with self._lock:
            self.is_LLM_enabled = False
        self.get_logger().info("VoiceConfirmation action started.")
        request = goal_handle.request.request
        duration = goal_handle.request.duration

        if request =="OpenDoor":
            key_words = ["open", "door"]
            audio_file= AUDIO_PATH + "OpenDoor.wav"
        elif request=="GetFood":
            key_words = ["get", "food"]
            audio_file= AUDIO_PATH + "GetFood.wav"
        elif request=="CloseDoor":
            key_words = ["close", "door"]
            audio_file= AUDIO_PATH + "CloseDoor.wav"
        else:
            self.get_logger().warn(f"Unknown request received in action: {request}")
            goal_handle.abort()
            return VoiceConfirmation.Result()

        subprocess.run(['pulseaudio','--start'])
        subprocess.run(['aplay','-D','hw:0,3','-f','S16_LE','-r','44100','-c','2','-d','1','/dev/zero'])
        subprocess.run(['aplay',audio_file])

        def timeout_callback():
            self.timeout_flag = True
            self.get_logger().info("VoiceConfirmation action timed out.")
        timer = self.create_timer(duration, timeout_callback)

        while not self.timeout_flag:
            if any([word in self.text for word in key_words]):
                self.get_logger().info(f"VoiceConfirmation action succeeded with text: {self.text}")
                goal_handle.succeed()
                break
        timer.cancel()

        if self.timeout_flag:
            goal_handle.abort()

        with self._lock:
            self.is_LLM_enabled = True
            self.timeout_flag = False
            self.text=""
        return VoiceConfirmation.Result()


    def cb_edge(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        # first time => always process
        if not self.edge_received_once:
            self.edge_received_once = True
            self.edge_last = text
            self.on_edge_update(text)
            return
        # changed => process
        if text != self.edge_last:
            self.edge_last = text
            self.on_edge_update(text)


    def cb_command(self, msg: String):
        with self._lock:
            self.text = msg.data.strip()
            if not self.text:
                self.text=""
                return     
            LLM_used = self.is_LLM_enabled
        if not LLM_used:
            return
        # first time => always process
        if not self.cmd_received_once:
            self.cmd_received_once = True
            self.cmd_last = self.text
            self.on_command_update(self.text)
            return
        # changed => process
        if self.text != self.cmd_last:
            self.cmd_last = self.text
            self.on_command_update(self.text)


    def on_edge_update(self, edge_text: str):
        """
        Called only when /edge_list is received first time or changed.
        """
        edges_nm_ps = [s.strip() for s in edge_text.splitlines() if s.strip()]
        
        self.edges = {}

        if len(edges_nm_ps) != 4:
            self.get_logger().warn(f"Unexpected /edge_list format: {edges_nm_ps}")
            return
        
        for edge_nm_ps in edges_nm_ps:
            edge_name = edge_nm_ps.split("//")[0]
            edge_ps = (ast.literal_eval(edge_nm_ps.split("//")[-1]))
            self.edges.update({edge_name : edge_ps})

        self.get_logger().info("------------- Edge updated --------------")
        for edge_n, edge_p in self.edges.items():        
            self.get_logger().info(f"  Edge name: {edge_n} pose {edge_p}")

        self.get_logger().info("-----------------------------------------\n")


    def on_command_update(self, command_text: str):
        if self.edges is None:
            return

        self.get_logger().info(f"received command {command_text}")
        out_LLM = self.LLM(command_text,"pose_select_both_examples_light")
        self.result_edge(out_LLM)
       

        self.get_logger().info(f"LLM output: {out_LLM}")

        if self.out_edge is None:
            self.get_logger().warn(f"No edge was detected: {command_text}")
        else:
            self.create_arrow_marker("selected pose", "map")

            trigger = Empty()
            out_LLM_msg = String()
            out_LLM_msg.data = self.out_edge + " // " + out_LLM
            

            self.LLM_out.publish(out_LLM_msg)
            self.prediction.publish(trigger)
            self.get_logger().info(f"prediction trigger")
        

    def both_ready(self) -> bool:
        return self.edge_received_once and self.cmd_received_once
        
        
    def LLM(self, text , prompt_key):

        base_prompt= self.prompts[prompt_key]
        prompt = base_prompt + text

        if self.source=="Open Router":
            API_URL = "https://openrouter.ai/api/v1/chat/completions"
            headers = {"Authorization": f"Bearer {API_KEY_OR}","Content-Type": "application/json"}
            data = {"model": "google/gemma-3n-e4b-it:free","messages": [{"role": "user", "content": prompt}]}
            
            result = requests.post(API_URL, json=data, headers=headers)

            try:
                output = result["choices"][0]["message"]["content"]
            except KeyError:
                print("KeyError in result, full content:", result)

        else:
            client = genai.Client(api_key=API_KEY)
            response = client.models.generate_content(model="gemma-3n-e4b-it", contents=prompt)
            output = response.text
        

        print("Assistant:", output)
        return output
 

    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def result_edge(self, LLM_out):

        self.out_edge = None
        self.out_edge_pose = None

   
        LLM_lower = LLM_out.split(".")[-2].casefold()  #Picking the last sentence of the LLM response

        for labels in self.edges.keys():
            parts = [p.strip() for p in labels.split(" / ")]
            for part in parts:
                pattern = rf"\b{re.escape(part.casefold())}\b"
                if re.search(pattern, LLM_lower):
                    self.out_edge = part
                    self.out_edge_pose = self.edges[labels]
                    return

        if self.out_edge is None or self.out_edge_pose == None:
            self.get_logger().warn(f"""No matching edge found in LLM output: "{LLM_lower}" """)
        else:
            self.get_logger().info(f"(Matching Edge: {self.out_edge}: {self.out_edge_pose})")


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def create_arrow_marker(self, name, frame_id="map"):
        marker = Marker()

        marker.header.frame_id = frame_id
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0

        marker.ns = "rso2_arrows"
        marker.id = hash(name) % 10000

        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # ✅ position from poses[name]
        marker.pose.position.x = self.out_edge_pose[0]
        marker.pose.position.y = self.out_edge_pose[1]
        marker.pose.position.z = 0.0

        # ✅ orientation from yaw
        
        q=self.yaw_to_quaternion(self.out_edge_pose[2])
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1] 
        marker.pose.orientation.z = q[2] 
        marker.pose.orientation.w = q[3] 

        # Arrow size
        marker.scale.x = 0.6   # length
        marker.scale.y = 0.1   # width
        marker.scale.z = 0.1   # height

        # Color (RGBA)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5

        marker.lifetime.sec = 3  # forever

        self.cposes_pub.publish(marker)

    def yaw_to_quaternion(eslf, yaw: float) -> list:
        """Convert yaw angle (rad) to quaternion."""
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        q=[qx,qy,qz,qw]
        return q

def main():
    rclpy.init()
    node = TaskManager()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()








