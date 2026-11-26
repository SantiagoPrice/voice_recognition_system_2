#!/usr/bin/env python3

#ros2 topic pub /command std_msgs/msg/String "{data: 'Right Edge'}"

import sys
import os

print("Node path:", os.getcwd())
import vosk
import json
import sounddevice as sd
import soundfile as sf
import queue
import time 
import numpy as np
import math
import rclpy
from rclpy.node import Node
from vosk import Model, GpuInit
from std_msgs.msg import String , UInt8MultiArray
from ament_index_python.packages import get_package_share_directory
import yaml

import threading



pkg_path = get_package_share_directory('vint_ros')
PAR_PATH = os.path.join(pkg_path,'conf/conf.yaml')
AUDIO_PATH = os.path.join(pkg_path,'others/audio_samples/others')

with open(PAR_PATH, "r") as f:
    conf_handlr= yaml.safe_load(f)
    MODEL_PATH = conf_handlr["vosk_model"]
    IN_STD = conf_handlr["input_dev"]
    OUT_STD = conf_handlr["output_dev"]




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
        sd.default.device = (IN_STD,OUT_STD)
        self.wake_word = "assistant"
        self.listening_for_command = False
        self.stop = False
        self.flag = 0
        self.pub_text = self.create_publisher(String, "/command", 10)
        

        audio_file=os.path.join(AUDIO_PATH,"button-pressed-sound.mp3")
        fback_audio, sf_fback = sf.read(audio_file)
        self.fback_audio = fback_audio
        self.sf_fback = sf_fback
        

        # Create a subscriber for button
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            '/ESP32/raw/mic', 
            self.listener_callback,
            10
        )

        self.read = False
        self.mutex=threading.Lock()

        print("===================================================")


    # Callback function for subscriber
    def listener_callback(self, msg):
        if not msg.data:
            return
        sd.play(self.fback_audio, self.sf_fback)
        sd.wait()
        with self.mutex:
            self.read=True
        
        print("Button pressed")


    def run(self):
        samplerate = int(sd.query_devices(None, "input")["default_samplerate"])
        print(f"actual sample rate is {samplerate}")
        with sd.RawInputStream(samplerate=samplerate, blocksize=int(samplerate*0.01), dtype="int16",
                               channels=1, callback=audio_callback):
            rec = vosk.KaldiRecognizer(model, samplerate)
            self.get_logger().info(f"Ready")
            print("Listening...")
            self.read=False

            while True:
                if self.stop == True:
                    continue
                data = q.get()

                
                if threshold_mode:
                    audio_np = np.frombuffer(data, dtype=np.int16)
                    volume =  np.max(np.abs(audio_np))/ 32768.0
                else:
                    volume = np.inf #skip threshold
                
                if volume < 0.15: # threshold_of_volume
                        pass
                else:
                    if threshold_mode:
                        self.read=True
                    with self.mutex:    
                        if self.read:
                            print(f"Volume: {volume:.4f}")

                if not self.read:
                    continue
                
                if rec.AcceptWaveform(data):

                    self.read=False
                    result = json.loads(rec.Result())
                    text = result.get("text", "").strip().lower()
                    rec.FinalResult()
                    if text:
                        self.get_logger().info(f"audio recording: {text}")
                        command = String()
                        command.data = text
                        self.pub_text.publish(command)
                    


def main_threshold_mode():
    rclpy.init()
    voice = VoiceRecognition()
    voice.run() 


def main_button_mode(args=None):
    rclpy.init(args=args)
    voice = VoiceRecognition()

    try:
        thread = threading.Thread(target=voice.run)
        thread.start()
        rclpy.spin(voice)
    except KeyboardInterrupt:
        pass
    finally:
        voice.destroy_node()
        rclpy.shutdown()
            

if __name__ == "__main__":
        # main()
    # Please set "threshold_mode" to "True" if you use threshold.
    threshold_mode = False
    if threshold_mode:
        main_threshold_mode()
    else:
        main_button_mode()
