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
import pandas as pd
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
from openpyxl import Workbook, load_workbook
from datetime import datetime
import yaml
import re



#pkg_path = get_package_share_directory('vint_ros')
pkg_path="/mnt/orin_ssd/workspaces/isaac_ros-dev/src/voice_recognition_system"

DB_PATH = os.path.join(pkg_path,'others/data_base.yaml')
PROMPT_PATH = os.path.join(pkg_path,'others/prompts.yaml')
API_PATH =  os.path.join(pkg_path,'others/conf.yaml')

with open(DB_PATH, "r") as f:
    db_handlr= yaml.safe_load(f)["db_test"]

with open(PROMPT_PATH, "r") as f:
    prompt_handlr= yaml.safe_load(f)

with open(API_PATH, "r") as f:
    conf_handlr= yaml.safe_load(f)
    API_KEY = ' '
    API_KEY_OR = conf_handlr["key_or"]




# voice_recognition
class VoiceRecognition():
    def __init__(self):
        print("===================================================")
        self.prompts=prompt_handlr
        self.db=db_handlr
        self.source="Open Router"
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def run(self):     
        TF = "origin"
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"prompt_test_{timestamp}.xlsx"
        header = ["Answer Label", "Request", "LLM Output", "Judged Label", "True/False"]

        wb = Workbook()
        ws = wb.active
        ws.append(header)
        wb.save(filename)

        for lab , exs in self.db.items():
            wb = load_workbook(filename)
            ws = wb.active
            print(f"label{lab}")

            for text in exs:

                out_LLM = self.LLM(self.LLM(text,"pose_select_both_examples_light_IDK_exs"))
                judged_label=self.result_edge(out_LLM)
                TF= judged_label == lab

                ws.append([lab, text, out_LLM, judged_label, TF])

                wb.save(filename)

                print("------------------------")
                time.sleep(10)

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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

            # else:
            #     client = genai.Client(api_key=API_KEY)
            #     response = client.models.generate_content(model="gemma-3n-e4b-it", contents=prompt)
            #     output = response.text
            

            print("Assistant:", output)
            return output
 

    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def result_edge(self, LLM_out):

        self.out_edge = None
   
        LLM_lower = LLM_out.split(".")[-2].casefold()  #Picking the last sentence of the LLM response

        for labels in self.edges.keys():
            parts = [p.strip() for p in labels.split(" / ")]
            for part in parts:
                pattern = rf"\b{re.escape(part.casefold())}\b"
                if re.search(pattern, LLM_lower):
                    self.out_edge = part

                    return

        if self.out_edge is None:
            self.get_logger().warn(f"""No matching edge found in LLM output: "{LLM_lower}" """)
        else:
            self.get_logger().info(f"(Matching Edge: {self.out_edge}")

##############################################################
def main():
    voice = VoiceRecognition()
    voice.run()

##############################################################
if __name__ == "__main__":
    main()
