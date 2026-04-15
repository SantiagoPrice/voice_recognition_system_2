#!/usr/bin/env python3

#ros2 topic pub /command std_msgs/msg/String "{data: 'Right Edge'}"

import sys
import os

print("Node path:", os.getcwd())
import vosk
import json
import sounddevice as sd
import queue
import time 
import numpy as np
import math
import rclpy
from rclpy.node import Node
from vosk import Model, GpuInit
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import yaml



pkg_path = get_package_share_directory('vint_ros')
PAR_PATH = os.path.join(pkg_path,'conf/conf.yaml')


with open(PAR_PATH, "r") as f:
    conf_handlr= yaml.safe_load(f)
    MODEL_PATH = conf_handlr["vosk_model"]
    DEV__N = conf_handlr["audio_dev"]



GpuInit()

model = Model(MODEL_PATH)

q = queue.Queue()

def audio_callback(indata, frames, time, status):
    if status:
        print(status, file = sys.stderr)
    q.put(bytes(indata))

class VoiceRecognition(Node):
    def __init__(self):
        super().__init__("voice")
        self.wake_word = "assistant"
        self.listening_for_command = False
        self.stop = False
        self.flag = 0
        self.pub_text = self.create_publisher(String, "/command", 10)
        print("===================================================")

    def run(self):
        samplerate = int(sd.query_devices(None, "input")["default_samplerate"])
        print(f"actual sample rate is {samplerate}")
        with sd.RawInputStream(samplerate=samplerate, blocksize=int(samplerate*0.01), dtype="int16",
                               channels=1, callback=audio_callback, device=DEV__N):
            rec = vosk.KaldiRecognizer(model, samplerate)
            self.get_logger().info(f"Ready")
            print("Listening...")
            self.read=False

            while True:
                if self.stop == True:
                    continue
                data = q.get()
                audio_np = np.frombuffer(data, dtype=np.int16)
                volume =  np.max(np.abs(audio_np))/ 32768.0
           
                # threshold_of_volume
                if volume < 0.15:
                        # print("Listening...")
                        pass
                else:
                    self.read=True
                    print(f"Volume: {volume:.4f}")
                    #result = json.loads(rec.Result())
                    #text = result.get("text", "").strip().lower()
                    #self.get_logger().info(text)

                if not self.read:
                    continue
                
                if rec.AcceptWaveform(data):
                #if self.read:
                    self.read=False
                    result = json.loads(rec.Result())
                    text = result.get("text", "").strip().lower()
                    rec.FinalResult()
                    #text = input("test sentence: ")
                    #print(f"audio recording: {text}")
                    if text:
                        self.get_logger().info(f"audio recording: {text}")
                        command = String()
                        command.data = text
                        self.pub_text.publish(command)
                    
    
def main():
    rclpy.init()
    voice = VoiceRecognition()
    voice.run() 

if __name__ == "__main__":
        main()
