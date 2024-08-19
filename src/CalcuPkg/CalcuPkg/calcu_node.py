import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt8
import math as m
import numpy as np

class CalcuNode(Node):
    def __init__(self):
        super().__init__("CalcuNode")

        self.postion = [0]*9

        self.motion_mode = 0
        self.target_xpos = [0,0,-1, 0,  0,-1, 0,  0,0]
        self.target_ypos = [0,1, 0, 0,  0, 1, 2,  0,0]
        self.target_deg  = [0,0, 0,90,-90, 0, 0,170,0]

        self.count = 0
        self.minite_error = 0.005
        self.deg_error = 0.5

        self.Vmax = 1.0
        self.A = 0.5
        self.T = [0,0,0]
        self.L = [0,0,0]

        self.Omega_max = 90
        self.angle_velo = 45
        self.T2 = [0,0,0]
        self.Theta = [0,0,0]

        self.R = 0
        self.Sita = 0
        self.Vs = 0

        self.theta = 0
        self.Omega_s = 0

        self.V = self.Vs
        self.Omega = self.Omega_s

        self.pos_sub = self.create_subscription(Float32MultiArray,"pos_topic",self.get_postion,10)
        self.auto_sub = self.create_subscription(UInt8,"auto_topic",self.auto_route,10)
        
        self.serial_send = Float32MultiArray()
        for i in range(8):
            self.serial_send.data.append(0)
        self.serial_pub = self.create_publisher(Float32MultiArray,"send_topic",10)

        self.route = self.create_timer(0.01,self.make_route)
        self.timer = self.create_timer(0.001,self.publish)
        
    def get_postion(self,msg):
        for i in range(9):
            self.postion[i] = msg.data[i]
        #print(self.postion)
    
    def auto_route(self,msg):
        self.motion_mode = msg.data
        print("start")

    def make_route(self):
        self.R = m.sqrt(m.pow(self.target_xpos[self.motion_mode]-self.postion[0],2)+m.pow(self.target_ypos[self.motion_mode]-self.postion[1],2))
        self.Sita = m.atan2(self.target_ypos[self.motion_mode]-self.postion[1],self.target_xpos[self.motion_mode]-self.postion[0])

        self.Vs = m.sqrt(m.pow(self.postion[3],2)+m.pow(self.postion[4],2))

        self.T[0] = (self.Vmax-self.Vs)/self.A
        self.T[2] = (self.Vmax)/self.A

        self.L[0] = 0.5*(self.Vs+self.Vmax)*self.T[0]
        self.L[2] = 0.5*(self.Vs)*self.T[2]

        self.T[1] = (self.R-self.L[0]-self.L[2])/self.Vmax

        if self.T[1] < 0:
            self.T[0] -= 0.5*self.T[1]
            self.T[2] -= 0.5*self.T[1]
            self.T[1] = 0


        self.theta = self.target_deg[self.motion_mode]-self.postion[2]
        self.Omega_s = self.postion[5]
        
        self.T2[0] = (self.Omega_max-self.Omega_s)/self.angle_velo
        self.T2[2] = (self.Omega_max)/self.angle_velo

        self.Theta[0] = 0.5*(self.Omega_s+self.Omega_max)*self.T2[0]
        self.Theta[2] = 0.5*(self.Omega_s)*self.T2[2]

        self.T2[1] = (self.theta-self.Theta[0]-self.Theta[2])/self.Vmax

        if self.T2[1] < 0:
            self.T2[0] -= 0.5*self.T[1]
            self.T2[2] -= 0.5*self.T[1]
            self.T2[1] = 0
        self.count = 0

    def publish(self):
        if self.count < self.T[0]:
            self.V = self.Vs + self.count*self.A
        elif self.count < self.T[0]+self.T[1]:
            self.V = self.V
        elif self.count < self.T[0]+self.T[1]+self.T[2]:
            self.V -= self.A*0.0001
        
        if self.count < self.T2[0]:
            self.Omega = self.Vs + self.count*self.angle_velo
        elif self.count < self.T2[0]+self.T2[1]:
            self.Omega = self.Omega
        elif self.count < self.T2[0]+self.T2[1]+self.T2[2]:
            self.Omega -= self.angle_velo*0.0001

        self.Vx = self.V*m.cos(self.Sita) * 1000.0
        self.Vy = self.V*m.sin(self.Sita) * 1000.0

        self.serial_send.data[0] = 0
        self.serial_send.data[1] = self.Vx
        self.serial_send.data[2] = self.Vy
        self.serial_send.data[3] = self.Omega
        self.serial_send.data[4] = self.postion[2]
        self.serial_send.data[5] = 0
        self.serial_send.data[6] = 0
        self.serial_send.data[7] = 0

        self.count += 0.0001
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