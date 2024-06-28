import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

class CalcuNode(Node):
    def __init__(self):
        super().__init__("CalcuNode")

        self.postion = [0]*3

        self.serial_sub = self.create_subscription(Float32MultiArray,"pos_topic",self.get_postion,10)
        
    def get_postion(self,msg):
        for i in range(3):
            self.postion[i] = msg.data[i]
        print(self.postion)

def main(args=None):
    rclpy.init(args=args)

    calcu_node = CalcuNode()
    rclpy.spin(calcu_node)
    calcu_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()