import sys
import os
import json
import vosk
import sounddevice as sd
import queue
import threading
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
from PyQt5.QtCore import QThread, pyqtSignal

#command list
COMMAND_LIST = [
    "browser - opne the browser",
    "terminal - create new pane",
    "list files - show the file list",
    "shutdown - shutdown the system",
    "bed - move to the bed",
    "table - move to the tabel"
]

#
MODEL_PATH = "vosk-model"
model = vosk.Model(MODEL_PATH)

#
q = queue.Queue()
def audio_callback(indata, frames, time, status):
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))

#ros2 node

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
        self.declare_parameter("GOAL","default_value")
        self.set_parameters([Parameter('GOAL', Parameter.Type.STRING, goal)])
        self.get_logger().info(f"Declared Parameter 'GOAL' with value: {goal}")

class ParamNode(Node):
    "Node to declare a parameters"
    def __init__(self):
        super().__init__('param_node')
        params = ["GOAL"]
        for param in params:
            self.declare_parameter("GOAL", "")


#
def launch_ros_node(node_class,spin=False):
    def target():
        rclpy.init()
        node = node_class()
        print("100")
        if spin:
            rclpy.spin(node) #keep the node alive
        else:
            rclpy.spin_once(node, timeout_sec=1.0)
        print("200")
        node.destroy_node()
    threading.Thread(target=target, daemon=True).start()



#voice recog

class VoiceRecognitionThread(QThread):
    detected_word = pyqtSignal(str)
    wake_word = "assistant"
    listening_for_command = False

    def run(self):
        samplerate = int(sd.query_devices(None, "input")["default_samplerate"])
        with sd.RawInputStream(samplerate=samplerate, blocksize=8000, dtype="int16",
                               channels=1, callback=audio_callback, device=None):
            rec = vosk.KaldiRecognizer(model, samplerate)
            print("Listening for wake word...")
            self.detected_word.emit("Waiting for wake word...")

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
                                self.detected_word.emit("Wake word detected! Please say a command...")
                                self.listening_for_command = True
                        else:
                            self.process_command(text)
                            self.listening_for_command = False
                            self.detected_word.emit("Waiting for wake word...")

    def process_command(self, text):
        text = "table"
        if "browser" in text:
            launch_ros_node(BrowserNode)
            self.detected_word.emit("Started BrowserNode")
        elif "terminal" in text:
            launch_ros_node(TerminalNode)
            self.detected_word.emit("Started TerminalNode")
        elif "list files" in text:
            launch_ros_node(ListFilesNode)
            self.detected_word.emit("Started ListFilesNode")
        elif "shutdown" in text:
            launch_ros_node(ShutdownNode)
            self.detected_word.emit("Started ShutdownNode")
        elif "bed" in text:
            launch_ros_node(BedNode)
            self.detected_word.emit("Started BedNode") 
        elif "table" in text:
            launch_ros_node(TableNode)
            self.detected_word.emit("Started TableNode")
        else:
            self.detected_word.emit("Command not recognized")

#GUI

class VoiceAssistantGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.voice_thread = VoiceRecognitionThread()
        self.voice_thread.detected_word.connect(self.update_label)
        self.voice_thread.start()
        launch_ros_node(ParamNode, spin=True) #initiate parameter node

    def initUI(self):
        self.layout = QVBoxLayout()
        self.status_label = QLabel("Offline voice recognition running...")
        self.layout.addWidget(self.status_label)

        self.commands_label = QLabel("Available Commands:\n" + "\n".join(COMMAND_LIST))
        self.layout.addWidget(self.commands_label)

        self.setLayout(self.layout)
        self.setWindowTitle("Offline Voice Assistant with ROS 2")
        self.setGeometry(100, 100, 500, 300)

    def update_label(self, text):
        self.status_label.setText(text)

#

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = VoiceAssistantGUI()
    gui.show()
    rclpy.shutdown()
    sys.exit(app.exec_())

