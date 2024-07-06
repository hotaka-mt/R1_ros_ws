import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt8
import numpy as np

class CalcuNode(Node):
    def __init__(self):
        super().__init__("CalcuNode")

        self.postion = [0]*3

        self.motion_mode = 0
        self.target_xpos = [0,0,-0.3, 0,  0,-1, 0,  0,0]
        self.target_ypos = [0,1,   0, 0,  0, 1, 2,  0,0]
        self.target_deg  = [0,0,   0,90,-90, 0, 0,170,0]

        self.count = 0
        self.minite_error = 0.005
        self.deg_error = 0.5
        self.timer_flag = True

        self.pos_sub = self.create_subscription(Float32MultiArray,"pos_topic",self.get_postion,10)
        self.auto_sub = self.create_subscription(UInt8,"auto_topic",self.auto_route,10)
        
        self.serial_send = Float32MultiArray()
        for i in range(8):
            self.serial_send.data.append(0)
        self.serial_pub = self.create_publisher(Float32MultiArray,"send_topic",10)
        
    def get_postion(self,msg):
        for i in range(3):
            self.postion[i] = msg.data[i]
        #print(self.postion)
    
    def auto_route(self,msg):
        self.motion_mode = msg.data
        print("start")
        if self.timer_flag: self.timer = self.create_timer(0.001,self.publish)

    def publish(self):
        self.timer_flag = False
        self.serial_send.data[0] = 0
        self.serial_send.data[1] = ((self.target_xpos[self.motion_mode]-self.postion[0])     *np.cos(np.radians(self.postion[2])) + (self.target_ypos[self.motion_mode]-self.postion[1])*np.sin(np.radians(self.postion[2])))*1000
        self.serial_send.data[2] = ((self.target_xpos[self.motion_mode]-self.postion[0])*(-1)*np.sin(np.radians(self.postion[2])) + (self.target_ypos[self.motion_mode]-self.postion[1])*np.cos(np.radians(self.postion[2])))*1000
        self.serial_send.data[3] = (self.target_deg [self.motion_mode]-self.postion[2])
        self.serial_send.data[4] = self.postion[2]
        self.serial_send.data[5] = 0
        self.serial_send.data[6] = 0
        self.serial_send.data[7] = 0
        if np.abs(self.target_xpos[self.motion_mode]-self.postion[0]) < self.minite_error and np.abs(self.target_ypos[self.motion_mode]-self.postion[1]) < self.minite_error and np.abs(self.target_deg[self.motion_mode]-self.postion[2]) < self.deg_error:
            #print("end")
            self.timer_flag = True
            for i in range(5):
                self.serial_send.data[i] = 0
            self.serial_send.data[5] = 0x01
            self.serial_pub.publish(self.serial_send)
            self.destroy_timer(self.timer)
            print(self.serial_send.data)
        print(self.serial_send.data)
        self.serial_pub.publish(self.serial_send)

def main(args=None):
    rclpy.init(args=args)

    calcu_node = CalcuNode()
    rclpy.spin(calcu_node)
    calcu_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()