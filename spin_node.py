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

class ParamNode(Node):
    def __init__(self):
        super().__init__('param_node')
        self.declare_parameter("GOAL", "")
        
        
if __name__ == "__main__":
    rclpy.init()
    node = ParamNode()
    rclpy.spin(node)

