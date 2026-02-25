#!/usr/bin/env python3

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
from vosk import Model, KaldiRecognizer , GpuInit
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory



GpuInit()

pkg_path = get_package_share_directory('vint_ros')


MODEL_PATH = os.path.join(pkg_path,"vosk-model-en-us-0.42-gigaspeech")
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
        with sd.RawInputStream(samplerate=samplerate, blocksize=int(samplerate*1), dtype="int16",
                               channels=1, callback=audio_callback, device=0):
            rec = vosk.KaldiRecognizer(model, samplerate)
            print("Listening...")
            self.read=False

            while True:
                if self.stop == True:
                    continue
                data = q.get()
                audio_np = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
                volume =  max(np.abs(audio_np))
           
                # threshold_of_volume
                if volume < 0.1:
                        print("Listening...")
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
