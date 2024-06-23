import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt8MultiArray
from SerialPkg import my_serial as serial

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.serial = serial.MySerial('/dev/ttyUSB0',115200)
        
        self.tx_data = [0]*15   #ROS->STM
        self.tx_select = [0xA0,0xA1,0xA2,0xA3,0xA4,0xA5]

        self.rx_bytes = [0]*39   #ROS<-STM
        self.rx_data = [0]*39   #ROS<-STM
        
        self.x_pos = 0
        self.y_pos = 0
        self.degree = 0
        self.condtion = 0
        self.publish_msg = UInt8MultiArray()
        for i in range(38):
            self.publish_msg.data.append(0)

        self.publisher = self.create_publisher(UInt8MultiArray,"reception_topic",10)
        self.subscriber = self.create_subscription(Float32MultiArray,"send_topic",self.make_send,10)
        self.timer = self.create_timer(0.01,self.interrupt)
    
    def interrupt(self):
        #ROSから送信
        self.serial.write(bytes(self.tx_data))
        #print(self.tx_data)
        
        #マイコンから受信しトピックに出力
        self.rx_bytes = self.serial.read([0xA5],40)
        for i in range(39):
            self.rx_data[i] = self.rx_bytes[i]
        if self.rx_data[38] != sum(self.rx_data[0:38]): return False

        for i in range(38):
            self.publish_msg.data[i] = self.rx_data[i]
        
        self.publisher.publish(self.publish_msg)
        print(self.rx_data)
        
    def make_send(self,msg):
        self.tx_data[0]  = self.tx_select[int(msg.data[0])]
        self.tx_data[1:5]  = serial.to_binary(msg.data[1])
        self.tx_data[5:9]  = serial.to_binary(msg.data[2])
        self.tx_data[9:13] = serial.to_binary(msg.data[3])
        self.tx_data[13] = int(msg.data[4])
        self.tx_data[14] = int(msg.data[5])

def main(args = None):
    rclpy.init(args=args)

    serial_node = SerialNode()

    rclpy.spin(serial_node)

    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()