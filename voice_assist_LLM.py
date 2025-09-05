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
from rcl_interfaces.msg import Parameter as MsgParameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from vosk import Model, KaldiRecognizer , GpuInit
from ament_index_python.packages import get_package_share_directory

# コマンドリスト
COMMAND_LIST = [
    "browser - open the browser",
    "terminal - create new pane",
    "list files - show the file list",
    "shutdown - shutdown the system",
    "bed - move to the bed",
    "table - move to the table"
]


GpuInit()

print([os.path.join(os.getcwd(),"/src/vint_ros/vosk-model")])

MODEL_PATH ="/home/orin/voice_recognition_system/vosk-model-lm/vosk-model-en-us-0.22"# ("/home/orin/ros2_ws/src/vint_ros/vosk-model")
#MODEL_PATH = "./src/vint_ros/vosk-model_big"
model = Model(MODEL_PATH)

#API key and URL
#API_KEY = ' '
API_KEY = ' '
API_URL = 'https://openrouter.ai/api/v1/chat/completions'

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

# ROS 2 ノード定義
class BrowserNode(Node):
    def __init__(self):
        super().__init__('browser_node')
        self.get_logger().info("Opening browser...")
        os.system("xdg-open https://www.google.com")

class TerminalNode(Node):
    def __init__(self):
        super().__init__('terminal_node')
        self.get_logger().info("Opening new terminal pane...")
        os.system("tmux split-window -h")

class ListFilesNode(Node):
    def __init__(self):
        super().__init__('list_files_node')
        self.get_logger().info("Listing files in terminal pane...")
        os.system("tmux split-window -h")
        os.system("tmux send-keys -t 1 'ls' Enter")

class ShutdownNode(Node):
    def __init__(self):
        super().__init__('shutdown_node')
        self.get_logger().info("Shutting down system via terminal...")
        os.system("tmux send-keys -t 1 'shutdown now' Enter")

class BedNode(Node):
    def __init__(self):
        goal = "59"
        super().__init__('bed_node')
        print("Started BedNode") 

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

class TableNode(Node):
    def __init__(self):
        goal = "60"
        super().__init__('table_node')
        print("Started TableNode") 
            
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


# ROSノード起動関数
def launch_ros_node(node_class, spin=False):
    def target():
        node = node_class()
        if spin:
            rclpy.spin(node)
        else:
            rclpy.spin_once(node, timeout_sec=1.0)
    threading.Thread(target=target, daemon=True).start()

# 音声認識処理
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
                start = time.time()
                data = q.get()
                end = time.time()
                time_diff = end - start
                #print("time1:",time_diff)

                audio_np = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
                volume =  max(np.abs(audio_np))#np.sqrt(np.mean(audio_np**2))  # RMSボリューム

            
                # 閾値以下ならvoskに渡さない
                if volume < 0.3:
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
                    #print("time2:",time_diff)
                    #text = "assistant"
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
            partial_result = json.loads(rec.PartialResult())
            print(f"Partial: {partial_result.get('partial', '')}")


    def LLM(self, text):
        #base_prompt =
        """
        You are an assistant AI with a power wheelchair.
        Please adhere to the following conditions and return output.

        - From the last sentence, think and decide where I want to go.
        - Select the destination from the "word list".
        - Select from the options as much as possible. 
        - If the destination is determined, the output should always be this sentence and nothing else.
        "Move to the "(you selected word)""
        - If the destination cannot be determined, the word "again" is output. 
        - If the assistant reads that there is nothing to do, the word "end" is output.

        word lists: "bed", "table"

        My request:

        """

        base_prompt = """
        You are a friendly AI assistant built into an electric wheelchair used in a bedroom. 
        The room has destinations: 
        - bed
        - table. 
        When the user speaks, always respond in one short, natural English sentence that clearly includes the destination.
        Do not ask questions. Instead, acknowledge the user’s state in a warm way and declare the action to move to the destination. 
        The response should sound natural when read aloud by a speaker. 
        If you get the word "assistant" from the User, you have to response "Hello, How can I help you."
        Examples: 
        User: “I’m hungry.” 
        Assistant: “Alright, let’s move to the table so you can eat.” 
        User: “I’m sleepy.” 
        Assistant: “Got it, heading to the bed now.”

        User: 
        """
        #text = "I am hungry"
        prompt = base_prompt + text
        print(prompt)
        data = {
                "model": "google/gemma-3n-e4b-it:free",
                "messages": [{"role": "user", "content": prompt}]
                }
        response = requests.post(API_URL, json=data, headers=headers)
        print(response)
        result = response.json()
        #print(result)
        output = result['choices'][0]['message']['content']
        print("Assistant:", output)
        engine.say(output)
        engine.runAndWait()
        return output

    def process_command(self, output):
        if "browser" in output:
            launch_ros_node(BrowserNode)
            print("Started BrowserNode")
            self.flag = 0
        elif "terminal" in output:
            launch_ros_node(TerminalNode)
            print("Started TerminalNode")
            self.flag = 0
        elif "list files" in output:
            launch_ros_node(ListFilesNode)
            print("Started ListFilesNode")
            self.flag = 0
        elif "shutdown" in output:
            launch_ros_node(ShutdownNode)
            print("Started ShutdownNode")
            self.flag = 0
        elif "bed" in output:
            launch_ros_node(BedNode)
            self.flag = 0
        elif "table" in output:
            launch_ros_node(TableNode)
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
