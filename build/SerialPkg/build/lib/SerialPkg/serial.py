import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import my_serial as serial

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.serial = serial.MySerial('/dev/ttyUSB0',115200)
        self.tx_data = [0]*14   #ROS->STM    1~4バイト:ｘ速度　5~8バイト:y速度　9~12バイト:角速度　13バイト：機構出力　14バイド：チェック用
        self.rx_data = [0]*14   #ROS<-STM    1~4バイト:ｘ位置　5~8バイト:y位置　9~12バイト:角度　　13バイト：機構状態　14バイド：チェック用
        
        self.x_pos = 0
        self.y_pos = 0
        self.radian = 0
        self.condtion = 0
        self.publish_msg = [0]*3

        self.publisher = self.create_publisher(Float32MultiArray,"reception_topic",10)
        self.subscriber = self.create_subscription(Float32MultiArray,"send_topic",self.make_send,10)
        self.timer = self.create_timer(0.01,self.interrupt)
    
    def interrupt(self):
        #ROSから送信
        self.serial.write([0xA5],bytes(self.tx_data))

        #マイコンから受信しトピックに出力
        self.rx_data = self.serial.read([0xA5],14)

        sum = 0
        for i in range(13):
            sum += self.rx_data[i]
        if sum&0xFF != self.rx_data[13]: return False

        self.x_pos  = serial.from_binary(self.rx_data[0:4],int)
        self.y_pos  = serial.from_binary(self.rx_data[4:8],int)
        self.radian = serial.from_binary(self.rx_data[8:12],int)
        self.condtion = self.rx_data[12]
        self.publish_msg = [self.x_pos,self.y_pos,self.condtion]
        self.publisher.publish(self.publish_msg)

    def make_send(self,msg):
        self.tx_data[0:4]  = serial.to_binary(msg[0])
        self.tx_data[4:8]  = serial.to_binary(msg[1])
        self.tx_data[8:12] = serial.to_binary(msg[2])
        self.tx_data[12] = msg[3]
        for i in range(13):
            self.tx_data[13] += self.tx_data[i]
        self.tx_data[13] = self.tx_data[13]&0xFF

if __name__ == "__main__":
    rclpy.init(args=None)

    serial_node = SerialNode()

    rclpy.spin(serial_node)

    serial_node.destroy_node()
    rclpy.shutdown()