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
from google import genai
import math
import random
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
from std_msgs.msg import Empty
from std_msgs.msg import Int32

API_KEY = ' '
API_URL = "https://openrouter.ai/api/v1/chat/completions"

client = genai.Client(api_key=" ")

engine = pyttsx3.init()
engine.setProperty("rate", 150)
engine.setProperty("volume", 1.0)

headers = {
        "Authorization": f"Bearer {API_KEY}",
        "Content-Type": "application/json"
        }

q = queue.Queue()


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


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
        
        # ---- subscriptions ----
        self.sub_edge = self.create_subscription(String, "/edge_list", self.cb_edge, 10)
        self.sub_cmd = self.create_subscription(String, "/command", self.cb_command, 10)
         
        # ---- publish ---- #
        self.prediction = self.create_publisher(Empty, "/prediction", 10)
        self.LLM_out = self.create_publisher(String, "/LLM_out", 10)
        

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
        text = msg.data.strip()
        if not text:
            return
        # first time => always process
        if not self.cmd_received_once:
            self.cmd_received_once = True
            self.cmd_last = text
            self.on_command_update(text)
            return
        # changed => process
        if text != self.cmd_last:
            self.cmd_last = text
            self.on_command_update(text)


    def on_edge_update(self, edge_text: str):
        """
        Called only when /edge_list is received first time or changed.
        """
        edges = [s.strip() for s in edge_text.splitlines() if s.strip()]
        self.edges = edges

        if len(edges) != 4:
            self.get_logger().warn(f"Unexpected /edge_list format: {edges}")
            return

        edge_1, edge_2, edge_3, edge_4 = edges
        self.get_logger().info("------------- Edge updated --------------")
        self.get_logger().info(f"  1: {edge_1}")
        self.get_logger().info(f"  2: {edge_2}")
        self.get_logger().info(f"  3: {edge_3}")
        self.get_logger().info(f"  4: {edge_4}")
        self.get_logger().info("-----------------------------------------\n")


    def on_command_update(self, command_text: str):
        if self.edges is None:
            return

        """
        self.get_logger().info(f"Command updated: {command_text}")
        out_LLM = self.LLM(command_text)
        """

        if any("Near edge" in e for e in self.edges):
            self.case = "case1"
            out_LLM = "OK, I will take you Right edge"
            #out_LLM = self.LLM_edge_case1(command_text)
            out_LLM = self.LLM_edge_case_both(command_text)
        elif any("Near right edge" in e for e in self.edges):
            self.case = "case2"
            out_LLM = "OK, I will take you Near Right edge"
            #out_LLM = self.LLM_edge_case2(command_text)
            out_LLM = self.LLM_edge_case_both(command_text)
        
        self.get_logger().info(f"LLM output: {out_LLM}")
        trigger = Empty()
        out_LLM_msg = String()
        out_LLM_msg.data = out_LLM
        self.LLM_out.publish(out_LLM_msg)
        self.prediction.publish(trigger)
        self.get_logger().info(f"prediction trigger")
        

    def both_ready(self) -> bool:
        return self.edge_received_once and self.cmd_received_once
        
        
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
        #response = requests.post(API_URL, json=data, headers=headers)
   

        response = client.models.generate_content(model="gemma-3n-e4b-it", contents=prompt)
        
        output = response.text
        #try:
        #    output = result["choices"][0]["message"]["content"]
        #except KeyError:
        #    print("KeyError in result, full content:", result)
        #    raise
        print("Assistant:", output)
        return output

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def LLM_edge_case1(self, text):
        base_prompt = """
        You are an assistant AI mounted on a wheelchair.
        You are currently near a rectangular table, and there are four possible positions around it:

        Near Edge – the side of the table closest to the wheelchair

        Far Edge – the side farthest from the wheelchair

        Right Edge – the right side of the table relative to the wheelchair’s forward direction

        Left Edge – the left side of the table relative to the wheelchair’s forward direction

        When the user makes a request, determine the most suitable position only if you can identify it with complete certainty.
        If there is any possibility of making an incorrect choice, you must respond with “I don't know.”

        Your response must follow these rules:

        Your output must include exactly one of the following options:
        “Near Edge”, “Far Edge”, “Right Edge”, “Left Edge”, or “I don't know”.

        Never output more than one of these labels in a single response.

        If you are not completely certain, respond only with “I don't know.”

        If you can determine the correct position, respond politely in one or two sentences and include the chosen label.

        User request:
        """
        prompt = base_prompt + text
        print(prompt)
        data = {
                "model": "google/gemma-3n-e4b-it:free",
                "messages": [{"role": "user", "content": prompt}]
                }
        #response = requests.post(API_URL, json=data, headers=headers)
   

        response = client.models.generate_content(model="gemma-3n-e4b-it", contents=prompt)
        
        output = response.text
        #try:
        #    output = result["choices"][0]["message"]["content"]
        #except KeyError:
        #    print("KeyError in result, full content:", result)
        #    raise
        print("Assistant:", output)
        return output

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def LLM_edge_case2(self, text):
        base_prompt = """
        You are an assistant AI mounted on a wheelchair.
        You are currently near a rectangular table, and there are four possible corner positions around it:

        Near Right Edge – near side + right side

        Far Right Edge – far side + right side

        Far Left Edge – far side + left side

        Near Left Edge – near side + left side

        When the user makes a request, determine the most suitable position only if you can identify it with complete certainty.
        If there is any risk of choosing the wrong position, you must respond with “I don't know.”

        Your response must follow these rules:

        Your output must include exactly one of the following options:
        “Near Right Edge”, “Far Right Edge”, “Far Left Edge”, “Near Left Edge”, or “I don't know”.

        Never output more than one of these labels in a single response.

        If you are not completely certain, respond only with “I don't know.”

        If you can determine the correct position, respond politely in one or two sentences and include the chosen label.

        User request:
        """
        prompt = base_prompt + text
        print(prompt)
        data = {
                "model": "google/gemma-3n-e4b-it:free",
                "messages": [{"role": "user", "content": prompt}]
                }
        #response = requests.post(API_URL, json=data, headers=headers)
   

        response = client.models.generate_content(model="gemma-3n-e4b-it", contents=prompt)
        
        output = response.text
        #try:
        #    output = result["choices"][0]["message"]["content"]
        #except KeyError:
        #    print("KeyError in result, full content:", result)
        #    raise
        print("Assistant:", output)
        return output



#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def LLM_edge_case_both(self, text):
        base_prompt = """
        You are an assistant AI mounted on a wheelchair.
        You are currently near a rectangular table, and two possible situations may happen:

        In the first situation you are directly facing one of the table edges. The edges names are the following ones:

        Near Edge – the side you are looking at
        Far Edge – the side farthest from the wheelchair
        Right Edge – the right side of the table relative to the wheelchair’s forward direction
        Left Edge – the left side of the table relative to the wheelchair’s forward direction

        In the second situation, the table is not perpendicular to you, but you still look towards the center of it.
        There are four possible edge positions around the table named based on the closest corner to you. The corner is shared by the near edges.

        Near Right Edge - Edge at the right side of the near corner  
        Near Left Edge – Edge at the left side of the near corner

        The furthest corner is localated in the opposite side of the nearest one. This corner is shared by the far edges
        Far Left Edge – Edge at the left side of the far corner
        Far Right Edge – Edge at the right side of the far corner

        When the user makes a request, infere which situation is happening and determine the most suitable position only if you can identify it with complete certainty.
        If there is any risk of choosing the wrong position, you must respond with “I don't know.”

        Your response must follow these rules:

        Your output must include exactly one of the following options if you think your arein the first situation:
        “Near Edge”, “Far Edge”, “Right Edge”, “Left Edge”,
         
        Your output must include exactly one of the following options if you think your arein the second situation:
        “Near Right Edge”, “Far Right Edge”, “Far Left Edge”, “Near Left Edge”,

        Never output more than one of these labels in a single response.
        If you are not completely certain, respond only with “I don't know.”
        If you can determine the correct position, respond politely in one or two sentences and include the chosen label.

        User request:
        """

        base_prompt2="""
        "You are an assistant AI mounted on a wheelchair, positioned near a rectangular table. Your goal is to accurately identify the requested table edge based on the user's instructions.

        Here's a reminder of the edge names:

        Situation 1: Facing a Table Edge Directly

        * Near Edge: The edge you are currently facing.
        * Far Edge: The edge directly opposite the Near Edge.
        * Right Edge: The right side of the table, relative to your forward direction.
        * Left Edge: The left side of the table, relative to your forward direction.

        Situation 2: Table Not Perpendicular - Viewing Towards Center

        * Near Right Edge: The right side of the near corner closest to you.
        * Far Right Edge: The right side of the far corner, opposite the near corner.
        * Far Left Edge: The left side of the far corner, opposite the near corner.
        * Near Left Edge: The left side of the near corner closest to you.

        Important Considerations:

        * The "back side" of the table refers to the edge directly opposite the edge you are facing.
        * When the table is not perpendicular, the "Near/Close/Front" and "Far/Back/Opposite" designations refer to the two corners closest and farthest from your current position, respectively.
        * If you are unsure about the user's request, respond with "I don't know."
        * Your output must include exactly one of the edges labels: “Near Edge”, “Far Edge”, “Right Edge”, “Left Edge”, “Near Right Edge”, “Far Right Edge”, “Far Left Edge”, “Near Left Edge”

        User Request:"
        """
        prompt = base_prompt2 + text
        print(prompt)
        data = {
                "model": "google/gemma-3n-e4b-it:free",
                "messages": [{"role": "user", "content": prompt}]
                }
        #response = requests.post(API_URL, json=data, headers=headers)
   

        response = client.models.generate_content(model="gemma-3n-e4b-it", contents=prompt)
        
        output = response.text
        #try:
        #    output = result["choices"][0]["message"]["content"]
        #except KeyError:
        #    print("KeyError in result, full content:", result)
        #    raise
        print("Assistant:", output)
        return output


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

def main():
    rclpy.init()
    node = TaskManager()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()








