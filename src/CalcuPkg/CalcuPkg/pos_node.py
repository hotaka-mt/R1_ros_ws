import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt8MultiArray
import numpy as np

class PosNode(Node):
    def __init__(self):
        super().__init__("pos_node")

        self.rs_data = [0]*3
        self.serial_data = [0]*4

        ### センサへの依存度を調整する。値が大きいほどRealSensenに、小さいほどトラッキングセンサに依存する
        self.alpha_x = 0.5
        self.alpha_y = 0.5
        self.alpha_deg = 0.5 
        ###

        self.x_pos = 0
        self.y_pos = 0
        self.deg = 0

        self.otos = [0]*3

        self.msg = Float32MultiArray()
        for i in range(3):
            self.msg.data.append(0)

        self.rs_sub = self.create_subscription(Float32MultiArray,"RS_topic",self.get_realsense,10)
        self.serial_sub = self.create_subscription(UInt8MultiArray,"reception_topic",self.get_serial,10)

        self.publisher = self.create_publisher(Float32MultiArray,"pos_topic",10)

    def get_realsense(self,msg):
        for i in range(3):
            self.rs_data[i] = msg.data[i]
        print(self.rs_data)
        self.calcu_position()

    def get_serial(self,msg):
        self.otos[0] = int(np.array(msg.data[1]<<8 | msg.data[0],dtype=np.int16)) * 0.0003
        self.otos[1] = int(np.array(msg.data[3]<<8 | msg.data[2],dtype=np.int16)) * 0.0003
        self.otos[2] = int(np.array(msg.data[5]<<8 | msg.data[4],dtype=np.int16)) * 0.0055
        print(self.otos)
        self.calcu_position()
    
    def calcu_position(self):
        self.x_pos = self.alpha_x*self.rs_data[0] + (1-self.alpha_x)*self.serial_data[0]
        self.y_pos = self.alpha_y*self.rs_data[1] + (1-self.alpha_y)*self.serial_data[1]
        self.deg = self.alpha_deg*self.rs_data[2] + (1-self.alpha_deg)*self.serial_data[2]
        self.msg.data = [self.x_pos,self.y_pos,self.deg]
        print("publish calcu_node")
        self.publisher.publish(self.msg)

def main(args = None):
    rclpy.init()

    pos_node = PosNode()
    rclpy.spin(pos_node)
    pos_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
