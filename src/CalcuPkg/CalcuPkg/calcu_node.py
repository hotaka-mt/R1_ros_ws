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
        self.target_xpos = [[ 0,-2.5,-2.5,-2.5,-2.5],[ 0, 2.5, 2.5, 2.5, 2.5]]
        self.target_ypos = [[ 0, 1.0, 1.5, 1.5, 1.0],[ 0, 1.0, 1.5, 1.5, 1.0]]
        self.target_deg  = [[ 0,   0,   0,   0,   0],[ 0,   0,   0,   0,   0]]

        self.count = 0
        self.minite_error = 0.01
        self.deg_error = 1
        self.Vmax = 0.5
        self.A = 0.5
        self.T = [0,0,0]
        self.L = [0,0,0]

        self.Omega_max = 180
        self.angle_accele = 360
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

        self.route = self.create_timer(0.1,self.make_route)
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
        self.L[2] = 0.5*(self.Vmax)*self.T[2]

        self.T[1] = (self.R-self.L[0]-self.L[2])/self.Vmax
        self.L[1] = self.Vmax*self.T[1]

        if self.T[1] < 0:
            self.T[0] += (self.Vmax/(self.Vmax+self.Vs))*self.T[1]
            self.T[2] += self.T[1]
            self.T[1] = 0

            self.L[0] = 0.5*(self.Vs+self.Vmax)*self.T[0]
            self.L[2] = 0.5*(self.Vmax)*self.T[2]
            self.L[1] = 0
            

        self.theta = self.target_deg[self.motion_mode]-self.postion[2]
        self.Omega_s = self.postion[5]
        
        self.T2[0] = (self.Omega_max-self.Omega_s)/self.angle_accele
        self.T2[2] = (self.Omega_max)/self.angle_accele

        self.Theta[0] = 0.5*(self.Omega_s+self.Omega_max)*self.T2[0]
        self.Theta[2] = 0.5*(self.Omega_max)*self.T2[2]

        self.T2[1] = (self.theta-self.Theta[0]-self.Theta[2])/self.Omega_max
        self.Theta[1] = self.Omega_max*self.T2[1]

        if self.T2[1] < 0:
            self.T2[0] += (self.Omega_max/(self.Omega_max+self.Omega_s))*self.T2[1]
            self.T2[2] += self.T2[1]
            self.T2[1] = 0

            self.Theta[0] = 0.5*(self.Omega_s+self.Omega_max)*self.T2[0]
            self.Theta[2] = 0.5*(self.Omega_max)*self.T2[2]
            self.Theta[1] = 0
        self.count = 0
        #print(f"{self.R} {self.Sita} {self.theta}")
        #print(f"{self.theta} {self.T2} {self.Theta}")

    def publish(self):
        if(self.R > self.minite_error):
            if self.count <= self.T[0]:
                self.V = self.Vs + self.count*self.A
            elif self.count <= self.T[0]+self.T[1]:
                self.V = self.V
            elif self.count <= self.T[0]+self.T[1]+self.T[2]:
                self.V -= self.A*0.001
            else:
                self.V = 0
        else:
            self.V = 0
        
        if(self.theta > self.deg_error):
            if self.count < self.T2[0]:
                self.Omega = self.Omega_s + self.count*self.angle_accele
            elif self.count < self.T2[0]+self.T2[1]:
                self.Omega = self.Omega
            elif self.count < self.T2[0]+self.T2[1]+self.T2[2]:
                self.Omega -= self.angle_accele*0.001
            else:
                self.Omega = 0
        elif(self.theta < -1*self.deg_error):
            if self.count < abs(self.T2[0]):
                self.Omega = self.Omega_s - self.count*self.angle_accele
            elif self.count < abs(self.T2[0]+self.T2[1]):
                self.Omega = self.Omega
            elif self.count < abs(self.T2[0]+self.T2[1]+self.T2[2]):
                self.Omega += self.angle_accele*0.001
            else:
                self.Omega = 0
        else:
            self.Omega = 0

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

        self.count += 0.001
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