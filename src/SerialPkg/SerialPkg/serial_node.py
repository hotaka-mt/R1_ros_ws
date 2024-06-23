import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from SerialPkg import my_serial as serial

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.serial = serial.MySerial('/dev/ttyUSB0',115200)
        self.tx_data = [0]*15   #ROS->STM
        self.rx_data = [0]*39   #ROS<-STM
        
        self.x_pos = 0
        self.y_pos = 0
        self.degree = 0
        self.condtion = 0
        self.publish_msg = Float32MultiArray()
        for i in range(3):
            self.publish_msg.data.append(0)

        self.publisher = self.create_publisher(Float32MultiArray,"reception_topic",10)
        self.subscriber = self.create_subscription(Float32MultiArray,"send_topic",self.make_send,10)
        self.timer = self.create_timer(0.01,self.interrupt)
    
    def interrupt(self):
        #ROSから送信
        self.serial.write(bytes(self.tx_data))
        print(self.tx_data)
        
        #マイコンから受信しトピックに出力
        '''
        self.rx_data = self.serial.read([0xA5],14)

        sum = 0
        for i in range(13):
            sum += self.rx_data[i]
        print(sum)
        if sum&0xFF != self.rx_data[13]: return False

        self.x_pos  = serial.from_binary(self.rx_data[0:4],int)
        self.y_pos  = serial.from_binary(self.rx_data[4:8],int)
        self.degree = serial.from_binary(self.rx_data[8:12],int)
        self.condtion = self.rx_data[12]
        self.publish_msg.data = [self.x_pos,self.y_pos,self.degree,self.condtion]
        print("publish serial_node")
        self.publisher.publish(self.publish_msg)
        '''

    def make_send(self,msg):
        #for i in range(6):
        #    print(f"{msg.data[i]}",end="    ")
        #print()
        self.tx_data[0]  = 0xA0
        self.tx_data[1:5]  = serial.to_binary(msg.data[0])
        self.tx_data[5:9]  = serial.to_binary(msg.data[1])
        self.tx_data[9:13] = serial.to_binary(msg.data[2])
        self.tx_data[13] = int(msg.data[3])
        self.tx_data[14] = int(msg.data[4])

def main(args = None):
    rclpy.init(args=args)

    serial_node = SerialNode()

    rclpy.spin(serial_node)

    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()