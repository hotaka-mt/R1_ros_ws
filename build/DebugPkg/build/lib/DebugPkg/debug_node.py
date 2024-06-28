import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import Int16MultiArray
import numpy as np

class DebugNode(Node):
    def __init__(self):
        super().__init__("debug_node")
        self.serial_send = Float32MultiArray()
        for i in range(6):
            self.serial_send.data.append(0)

        self.rs_data = [0]*3
        self.serial_rece_data = [0]*40
        self.otos_data = [0]*3
        self.im920_data = [0]*3
        self.pos_data = [0]*4

        self.rs_sub = self.create_subscription(Float32MultiArray,"RS_topic",self.get_realsense,10)
        self.serial_sub = self.create_subscription(UInt8MultiArray,"reception_topic",self.get_serial,10)
        self.IM920_sub = self.create_subscription(Int16MultiArray,"IM920_rece_topic",self.get_IM920,10)
        self.pos_sub = self.create_subscription(Float32MultiArray,"pos_topic",self.get_position,10)

        self.serial_pub = self.create_publisher(Float32MultiArray,"send_topic",10)
        self.IM920_pub = self.create_publisher(Float32MultiArray,"IM920_send_topic",10)

        self.tiemr = self.create_timer(0.001,self.interrupt)

        self.Param = [0]*64
        self.before_Param = [0]*64
        self.send_mode = 0
        self.gain = []

    def interrupt(self):
        self.Param = [0]*64
        with open("src/DebugPkg/DebugPkg/robot_param.md") as param:
            self.Param = param.read().split()
            if len(self.Param) == 0: return False

            if self.Param[30:36] != self.before_Param[30:36]: 
                self.mecanum_publish(float(self.Param[31]),float(self.Param[33]),float(self.Param[35]))
            
            #if self.Param[0:30] != self.before_Param[0:30]:
            #    for i in range(15): self.gain.append(self.Param[i*2+1])
            #    self.send_mode = 1
            #    self.param_set(self.gain)

            #if self.send_mode != 0: self.param_set(self.gain)

            for i in range(len(self.Param)):
                self.before_Param[i] = self.Param[i]

    def mecanum_publish(self,x,y,omega):
        #print(f"{x} {y} {omega}")
        self.serial_send.data[0] = 0
        self.serial_send.data[1] = float(x)
        self.serial_send.data[2] = float(y)
        self.serial_send.data[3] = float(omega)
        self.serial_send.data[4] = 0
        self.serial_send.data[5] = 0
        self.serial_pub.publish(self.serial_send)

    def param_set(self,gain):
        #print(self.head_byte)
        print(self.send_mode)
        self.param_publish(self.send_mode,gain[(self.send_mode-1)*3],gain[(self.send_mode-1)*3+1],gain[(self.send_mode-1)*3+2])
        if self.send_mode == self.serial_rece_data[6]-0xA0: self.send_mode += 1
        if self.send_mode >= 6: self.send_mode = 0

    def param_publish(self,num,P,I,D):
        self.serial_send.data[0] = float(num)
        self.serial_send.data[1] = float(P)
        self.serial_send.data[2] = float(I)
        self.serial_send.data[3] = float(D)
        self.serial_send.data[4] = 0
        self.serial_send.data[5] = 0
        self.serial_pub.publish(self.serial_send)

    def get_realsense(self,msg):
        #print(msg.data)
        for i in range(3):
            self.rs_data[i] = msg.data[i]
        self.update_robotparam()
    def get_serial(self,msg):
        #print(msg.data)
        for i in range(10):
            self.serial_rece_data[i] = msg.data[i]
        self.otos_data[0] = int(np.array(msg.data[1]<<8 | msg.data[0],dtype=np.int16)) * 0.0003
        self.otos_data[1] = int(np.array(msg.data[3]<<8 | msg.data[2],dtype=np.int16)) * 0.0003
        self.otos_data[2] = int(np.array(msg.data[5]<<8 | msg.data[4],dtype=np.int16)) * 0.0055
        #print(self.serial_rece_data)
        self.update_robotparam()
    def get_IM920(self,msg):
        #print(msg.data)
        for i in range(3):
            self.im920_data[i] = msg.data[i]
        self.update_robotparam()
    def get_position(self,msg):
        #print(msg.data)
        for i in range(3):
            self.pos_data[i] = msg.data[i]
        self.update_robotparam()
    
    def update_robotparam(self):
        with open("src/DebugPkg/DebugPkg/robot_param.md","w") as param:
            for i in range(18):
                param.write(self.Param[i*2]+"   "+self.Param[i*2+1]+"\n")
            for i in range(3):
                param.write(self.Param[36+i*2]+"   "+"{:.3f}".format(self.pos_data[i])+"\n")
            for i in range(3):
                param.write(self.Param[42+i*2]+"   "+"{:.3f}".format(self.rs_data[i])+"\n")
            for i in range(3):
                param.write(self.Param[48+i*2]+"   "+"{:.3f}".format(self.otos_data[i])+"\n")
            for i in range(5):
                param.write(self.Param[54+i*2]+"    "+f"{(self.serial_rece_data[7]&(1<<i))>>i}"+"\n")

def main(args = None):
    rclpy.init(args=args)

    debug_node = DebugNode()
    rclpy.spin(debug_node)
    debug_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
