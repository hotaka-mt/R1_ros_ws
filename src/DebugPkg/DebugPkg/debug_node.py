import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16MultiArray
from DebugPkg import debug_GUI

class DebugNode(Node):
    def __init__(self):
        super().__init__("debug_node")

        self.gui = debug_GUI.DebugGUI()

        self.rs_data = [0]*3
        self.serial_data = [0]*4
        self.im920_data = [0]*3
        self.pos_data = [0]*4

        self.rs_sub = self.create_subscription(Float32MultiArray,"RS_topic",self.get_realsense,10)
        self.serial_sub = self.create_subscription(Float32MultiArray,"reception_topic",self.get_serial,10)
        self.IM920_sub = self.create_subscription(Int16MultiArray,"IM920_rece_topic",self.get_IM920,10)
        self.pos_sub = self.create_subscription(Float32MultiArray,"pos_topic",self.get_position,10)

        self.serial_pub = self.create_publisher(Float32MultiArray,"send_topic",10)
        self.serial_pub = self.create_publisher(Float32MultiArray,"IM920_send_topic",10)

    def get_realsense(self,msg):
        for i in range(3):
            self.rs_data[i] = msg.data[i]
    def get_serial(self,msg):
        for i in range(4):
            self.serial_data[i] = msg.data[i]
    def get_IM920(self,msg):
        for i in range(3):
            self.im920_data[i] = msg.data[i]
    def get_position(self,msg):
        for i in range(3):
            self.pos_data[i] = msg.data[i]

def main(args = None):
    rclpy.init(args=args)

    debug_node = DebugNode()

    rclpy.spin(debug_node)
    debug_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
