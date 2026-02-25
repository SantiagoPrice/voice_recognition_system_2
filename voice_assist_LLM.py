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
from rclpy.node import Node
from rclpy.parameter import Parameter as RclpyParameter
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.msg import Parameter as MsgParameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from vosk import Model, KaldiRecognizer , GpuInit
from ament_index_python.packages import get_package_share_directory

GpuInit()

print([os.path.join(os.getcwd(),"/src/vint_ros/vosk-model")])

MODEL_PATH ="/home/orin/voice_recognition_system/vosk-model-lm/vosk-model-en-us-0.22"# ("/home/orin/ros2_ws/src/vint_ros/vosk-model")
#MODEL_PATH = "./src/vint_ros/vosk-model_big"
model = Model(MODEL_PATH)

#API key and URL
API_KEY = ' '
API_URL = 'https://openrouter.ai/api/v1/chat/completions'

_current_node = None
_current_thread = None
_sopt_event = None

#pyttsx3 srttings
engine = pyttsx3.init()
engine.setProperty("rate", 150)
engine.setProperty("volume", 1.0)

# Define the headers for the API request
headers = {
        'Authorization': f'Bearer {API_KEY}',
        'Content-Type': 'application/json'
        }

q = queue.Queue()
def audio_callback(indata, frames, time, status):
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))

# create_node
class CreateNode(Node):
    def __init__(self, target, goal):
        super().__init__(target)
        print(f"Started {target}")
        print("this is CreateNode")
            
        self.cli_tr = self.create_client(SetParameters, '/p1_trig_vint_node/set_parameters')
        self.cli_par = self.create_client(SetParameters, '/tracker_with_cloud_node/set_parameters')

        
        while not self.cli_par.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /p1_trig_vint_node/set_parameters service...')
        
        while not self.cli_tr.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /tracker_with_cloud_node/set_parameters service...')

        param = MsgParameter()
        param.name = 'GOAL'
        param.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=goal)
        req = SetParameters.Request()
        req.parameters = [param]
        self.future = self.cli_par.call_async(req)

        param.name = 'goal_trigg'
        param.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True)
        self.future = self.cli_tr.call_async(req)

        self.get_logger().info(f"Declared Parameter 'GOAL' with value: {goal}")


# launch_ROS_nod
"""e
def launch_ros_node(node_class, node_name, goal, spin=False):
    global _current_node, _current_thread, _stop_event

    if _current_thread is not None and _stop_event is not None:
        _stop_event.set()
        _current_thread.join()
        if _current_node is not None:
            _current_node.destroy_node()
        _current_executor = None
        _current_node = None
        _stop_event = None

    def target():
        global _current_node, _stop_event
        node = node_class(node_name, goal)
        _current_node = node
        stop_event = threading.Event()
        _stop_event = stop_event

        while not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.01)
    
    _current_thread = threading.Thread(target=target, daemon=True)
    _current_thread.start()
"""
def launch_ros_node(node_class, spin=False):
    def target():
        node = node_class()
        if spin:
            rclpy.spin(node)
        else:
            rclpy.spin_once(node, timeout_sec=1.0)
    threading.Thread(target=target, daemon=True).start()

# voice_recognition
class VoiceRecognition:
    def __init__(self):
        self.wake_word = "assistant"
        self.listening_for_command = False
        self.stop = False
        self.flag = 0
        print("===================================================")

    def run(self):
        #device_info = sd.query_devices(None, "input")
        #print(f"Default input device: {device_info['name']}")
        #print(f"Default sample rate: {device_info['default_samplerate']} Hz")
        samplerate = int(sd.query_devices(None, "input")["default_samplerate"])
        print(f"actual sample rate is {samplerate}")
        #samplerate2 = 16000
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
                volume =  max(np.abs(audio_np))#np.sqrt(np.mean(audio_np**2))  # RMSボリューム

            
                # threshold_of_volume
                if volume < 0.1:
                    if self.listening_for_command  == True:
                        print("Listening...")
                    else:
                     print("Listening for wake word...")
                     #continue
                else:
                    self.read=True
                    print(f"Volume: {volume:.4f}")
                
                if not self.read:
                    continue

                if rec.AcceptWaveform(data):
                    self.read=False
                    result = json.loads(rec.Result())
                    text = result.get("text", "").strip().lower()
                    text = input("test sentence: ")
                    print(f"audio recording: {text}")
                    if text:
                        print(f"(Detected: {text})")
                        if not self.listening_for_command and self.flag != 1:
                            if self.wake_word in text:
                                #self.LLM(text)
                                self.voice_feedback(text)
                                self.listening_for_command = True
                        else:
                            self.stop = True
                            output = self.LLM(text)
                            self.process_command(output)
                            self.listening_for_command = False
                            if self.flag == 1:
                                print("Listening...")
                else:
                    partial_result = json.loads(rec.PartialResult())
                    print(f"Partial: {partial_result.get('partial', '')}")

    def voice_feedback(self, text):
        if text == "assistant":
            print("Hello, How can I help you.")
            engine.say("Hello, How can I help you.")
            engine.runAndWait()
        else:
            print("Listening...")
            partial_result = json.loads(rec.Result())
            print(f"Partial: {partial_result.get('partial', '')}")


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
        engine.say(output)
        engine.runAndWait()
        return output

    def process_command(self, output):
        if "bed" in output:
            node_name = "bed_node"
            goal = "59"
            launch_ros_node(CreateNode, node_name, goal)
            self.flag = 0
        elif "table" in output:
            node_name = "table_node"
            goal = "60"
            launch_ros_node(CreateNode, node_name, goal)
            self.flag = 0
        elif "stop" in output:
            node_name = "stop_node"
            goal = "1"
            launch_ros_node(CreateNode, node_name, goal)
            self.flag = 0
        elif "again" in output:
            print("Please say again")
            self.flag = 1
        elif "end" in output:
            self.flag = 0
        else:
            self.flag = 2
        self.stop = False

rclpy.init()
recognizer = VoiceRecognition()
recognizer.run()

if __name__ == "__main__":
    main()
