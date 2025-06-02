import sys
import os
import json
import vosk
import sounddevice as sd
import queue
import threading
import rclpy
import time
from rclpy.node import Node
from rclpy.parameter import Parameter

# コマンドリスト
COMMAND_LIST = [
    "browser - open the browser",
    "terminal - create new pane",
    "list files - show the file list",
    "shutdown - shutdown the system",
    "bed - move to the bed",
    "table - move to the table"
]

MODEL_PATH = "vosk-model"
model = vosk.Model(MODEL_PATH)

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
        goal = "bed"
        super().__init__('bed_node')
        self.set_parameters([Parameter('GOAL', Parameter.Type.STRING, goal)])
        self.get_logger().info(f"Declared Parameter 'GOAL' with value: {goal}")

class TableNode(Node):
    def __init__(self):
        goal = "table"
        super().__init__('table_node')
        self.declare_parameter("GOAL", "default_value")
        self.set_parameters([Parameter('GOAL', Parameter.Type.STRING, goal)])
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

    def run(self):
        samplerate = int(sd.query_devices(None, "input")["default_samplerate"])
        with sd.RawInputStream(samplerate=samplerate, blocksize=8000, dtype="int16",
                               channels=1, callback=audio_callback, device=None):
            rec = vosk.KaldiRecognizer(model, samplerate)
            print("Listening for wake word...")

            while True:
                data = q.get()
                if rec.AcceptWaveform(data):
                    result = json.loads(rec.Result())
                    #text = result.get("text", "").strip().lower()
                    text = "assistant"
                    if text:
                        print(f"Detected: {text}")
                        if not self.listening_for_command:
                            if self.wake_word in text:
                                print("Wake word detected! Please say a command...")
                                self.listening_for_command = True
                        else:
                            self.process_command(text)
                            self.listening_for_command = False
                            print("Waiting for wake word...")

    def process_command(self, text):
        text = "table"
        if "browser" in text:
            launch_ros_node(BrowserNode)
            print("Started BrowserNode")
        elif "terminal" in text:
            launch_ros_node(TerminalNode)
            print("Started TerminalNode")
        elif "list files" in text:
            launch_ros_node(ListFilesNode)
            print("Started ListFilesNode")
        elif "shutdown" in text:
            launch_ros_node(ShutdownNode)
            print("Started ShutdownNode")
        elif "bed" in text:
            launch_ros_node(BedNode)
            print("Started BedNode") 
        elif "table" in text:
            launch_ros_node(TableNode)
            print("Started TableNode")
        else:
            print("Command not recognized")

if __name__ == "__main__":
    rclpy.init()
    recognizer = VoiceRecognition()
    recognizer.run()
