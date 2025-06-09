import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class ParamNode(Node):
    def __init__(self):
        super().__init__('param_node')
        self.declare_parameter("GOAL", "")
        
def main():
    rclpy.init()
    node = ParamNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
