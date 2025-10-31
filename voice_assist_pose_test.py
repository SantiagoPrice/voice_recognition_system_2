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

#####################################

labels_1 = ["Near Edge", "Right Edge", "Left Edge", "Far Edge"]

labels_2 = ["Near Right Edge", "Far Edge", "Left Edge", "Near Left Edge"]

count_false = np.zeros(4)

req = ["I want to go this side",
        "I want to go to the right",
        "I want to go to the left",
        "I want to go far",
        "I want to go somewhere close",
        "I want to go over to the right",
        "I want to go over to the left",
        "I want to move farther away",
        "I want to move closer",
        "I want to move toward the right side",
        "I want to move toward the left side",
        "I want to go a over there"
        ]



############################################################################

# voice_recognition
class VoiceRecognition():
    def __init__(self):
        self.wake_word = "assistant"
        self.listening_for_command = False
        self.stop = False
        self.flag = 0
        print("===================================================")
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def run(self):
        a = 100
        true = 0
        false = 0 
        for i in range(a):
            print(i+1)
            number = random.randint(0, 11)
            text = req[number]
            answer = number%4
            output = self.LLM_edge_case1(text)
            print("output succses")
            edge_num = self.result_edge(output, labels_1)
            if edge_num == answer:
                print(req[number])
                print(labels_1[answer])
                true += 1
                print("TRUE")
            if not edge_num == answer:
                false += 1
                count_false[answer] += 1
                print(req[number])
                print(labels_1[answer])
                print("FALSE")

            accuracy = (true / a) * 100
            print(accuracy)
            print(count_false)
            print("------------------------")
            time.sleep(10)

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def LLM_edge_case1(self, text):
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
        print(response)
        result = response.json()
        output = result["choices"][0]["message"]["content"]
        print("Assistant:", output)
        return output

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def LLM_edge_case2(self, text, edges):
        base_prompt = """You are an assistant AI mounted on a wheelchair.
        You are currently near a table, and there are four possible positions around it:
        "Near Right Edge"
        "Far Edge"
        "Left Edge"
        "Near Left Edge"
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
        print(response)
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
                print(edges[m])
                continue
        return edge_number

##############################################################
def main():
    voice = VoiceRecognition()
    voice.run()

##############################################################
if __name__ == "__main__":
    main()
