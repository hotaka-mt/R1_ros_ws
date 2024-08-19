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
        self.alpha_x = 0
        self.alpha_y = 0
        self.alpha_deg = 0
        ###

        self.x_pos = 0
        self.y_pos = 0
        self.deg = 0
        self.x_velo = 0
        self.y_velo = 0
        self.omega = 0
        self.x_accle = 0
        self.y_accle = 0
        self.angle_accle = 0

        self.otos = [0]*9

        self.msg = Float32MultiArray()
        for i in range(9):
            self.msg.data.append(0)

        self.rs_sub = self.create_subscription(Float32MultiArray,"RS_topic",self.get_realsense,10)
        self.serial_sub = self.create_subscription(UInt8MultiArray,"reception_topic",self.get_serial,10)
        
        self.publisher = self.create_publisher(Float32MultiArray,"pos_topic",10)

    def get_realsense(self,msg):
        for i in range(3):
            self.rs_data[i] = msg.data[i]
        #print(self.rs_data)
        self.calcu_position()

    def get_serial(self,msg):
        self.otos[0] = int(np.array(msg.data[2]<<8 | msg.data[1],dtype=np.int16)) * 0.0003
        self.otos[1] = int(np.array(msg.data[4]<<8 | msg.data[3],dtype=np.int16)) * 0.0003
        self.otos[2] = int(np.array(msg.data[6]<<8 | msg.data[5],dtype=np.int16)) * 0.0055
        self.otos[3] = int(np.array(msg.data[8]<<8 | msg.data[7],dtype=np.int16)) * 0.0015
        self.otos[4] = int(np.array(msg.data[10]<<8 | msg.data[9],dtype=np.int16)) * 0.0015
        self.otos[5] = int(np.array(msg.data[12]<<8 | msg.data[11],dtype=np.int16)) * 0.0061
        self.otos[6] = int(np.array(msg.data[14]<<8 | msg.data[13],dtype=np.int16)) * 0.0048
        self.otos[7] = int(np.array(msg.data[16]<<8 | msg.data[15],dtype=np.int16)) * 0.0048
        self.otos[8] = int(np.array(msg.data[18]<<8 | msg.data[17],dtype=np.int16)) * 5.5
        print(self.otos)
        self.calcu_position()
    
    def calcu_position(self):
        self.x_pos = self.alpha_x*self.rs_data[0] + (1-self.alpha_x)*self.otos[0]
        self.y_pos = self.alpha_y*self.rs_data[1] + (1-self.alpha_y)*self.otos[1]
        self.deg = self.alpha_deg*self.rs_data[2] + (1-self.alpha_deg)*self.otos[2]
        self.x_velo = self.otos[3]
        self.y_velo = self.otos[4]
        self.omega = self.otos[5]
        self.x_accle = self.otos[6]
        self.y_accle = self.otos[7]
        self.angle_accle = self.otos[8]
        self.msg.data = [self.x_pos,self.y_pos,self.deg,self.x_velo,self.y_velo,self.omega,self.x_accle,self.y_accle,self.angle_accle]
        #print("publish calcu_node")
        self.publisher.publish(self.msg)

def main(args = None):
    rclpy.init()

    pos_node = PosNode()
    rclpy.spin(pos_node)
    pos_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
