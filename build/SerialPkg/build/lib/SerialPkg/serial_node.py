import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt8MultiArray
import serial
import struct

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.serial = serial.Serial('/dev/ttyUSB0',115200)
        
        self.tx_data = [0]*15   #ROS->STM
        self.tx_select = [0xA0,0xA1,0xA2,0xA3,0xA4,0xA5]

        self.rx_byte = 0
        self.count = 0
        self.rx_data = [0]*41   #ROS<-STM
        
        self.x_pos = 0
        self.y_pos = 0
        self.degree = 0
        self.condtion = 0
        self.publish_msg = UInt8MultiArray()
        for i in range(10):
            self.publish_msg.data.append(0)

        self.publisher = self.create_publisher(UInt8MultiArray,"reception_topic",10)
        self.subscriber = self.create_subscription(Float32MultiArray,"send_topic",self.make_send,10)
        self.timer = self.create_timer(0.001,self.interrupt)
    
    def interrupt(self):
        #ROSから送信
        self.serial.write(bytes(self.tx_data))
        #print(self.tx_data)
        
        #マイコンから受信しトピックに出力
        self.rx_byte = self.serial.read(1)
        if self.rx_byte[0] == 0xA6: self.count = 0
        self.rx_data[self.count] = self.rx_byte[0]
        self.count += 1

        if self.rx_data[11] != sum(self.rx_data[1:11]) & 0xFF: 
            #print(sum(self.rx_data[1:12]) & 0xFF)
            return False
        print(self.rx_data[0:12])

        for i in range(1,11):
            self.publish_msg.data[i-1] = self.rx_data[i]
        
        self.publisher.publish(self.publish_msg)
        
    def make_send(self,msg):
        self.tx_data[0]  = self.tx_select[int(msg.data[0])]
        self.tx_data[1:5]  = self.to_binary(msg.data[1])
        self.tx_data[5:9]  = self.to_binary(msg.data[2])
        self.tx_data[9:13] = self.to_binary(msg.data[3])
        self.tx_data[13] = int(msg.data[4])
        self.tx_data[14] = int(msg.data[5])
        #print(self.tx_data)

    def to_binary(self,val):
        #int、float型をバイナリー変換
        if type(val) is int:
            binary = struct.pack('<i',val)
        elif type(val) is float:
            binary = struct.pack('<f',val)
        else:
            binary = None
        return binary


def main(args = None):
    rclpy.init(args=args)

    serial_node = SerialNode()

    rclpy.spin(serial_node)

    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()