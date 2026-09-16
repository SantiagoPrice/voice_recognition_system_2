#!/usr/bin/env python3

import os
import requests
import yaml
from google import genai

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

pkg_path = get_package_share_directory('vint_ros')
API_PATH = os.path.join(pkg_path, 'conf/conf.yaml')
PROMPT_PATH = os.path.join(pkg_path, 'others/prompts.yaml')

with open(API_PATH, "r") as f:
    conf_handlr = yaml.safe_load(f)
    API_KEY = conf_handlr["key"]
    API_KEY_OR = conf_handlr["key_or"]

with open(PROMPT_PATH, "r") as f:
    prompt_handlr = yaml.safe_load(f)


class TaskManager(Node):
    def __init__(self):
        super().__init__("task_manager")

        self.prompts = prompt_handlr
        # Which prompt template to use when building the LLM request.
        # Change this to whichever key exists in prompts.yaml.
        self.prompt_key = "pose_select_both_examples_light"

        if API_KEY:
            self.source = "Genai"
        elif API_KEY_OR:
            self.source = "Open Router"
        else:
            raise ValueError("NO API key")

        self.get_logger().info(f"LLM connected via {self.source}")

        self.sub_cmd = self.create_subscription(String, "/command", self.cb_command, 10)
        self.pub_llm_out = self.create_publisher(String, "/LLM_output", 10)

    def cb_command(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f"Received command: {text}")
        output = self.LLM(text, self.prompt_key)
        self.get_logger().info(f"LLM output: {output}")

        out_msg = String()
        out_msg.data = output
        self.pub_llm_out.publish(out_msg)

    def LLM(self, text, prompt_key):
        base_prompt = self.prompts[prompt_key]
        prompt = base_prompt + text

        if self.source == "Open Router":
            API_URL = "https://openrouter.ai/api/v1/chat/completions"
            headers = {"Authorization": f"Bearer {API_KEY_OR}", "Content-Type": "application/json"}
            data = {"model": "google/gemini-3.5-flash", "messages": [{"role": "user", "content": prompt}]}

            result = requests.post(API_URL, json=data, headers=headers).json()

            try:
                output = result["choices"][0]["message"]["content"]
            except KeyError:
                self.get_logger().error(f"KeyError in result, full content: {result}")
                output = ""
        else:
            client = genai.Client(api_key=API_KEY)
            response = client.models.generate_content(model="gemma-4-26b-a4b-it", contents=prompt)
            output = response.text

        return output


def main():
    rclpy.init()
    node = TaskManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()