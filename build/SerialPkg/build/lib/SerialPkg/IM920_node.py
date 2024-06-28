import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32MultiArray
from SerialPkg import IM920sL

class IM920Node(Node):
    def __init__(self):
        super().__init__ ('IM920_node')
        self.IM920 = IM920sL.IM920('/dev/ttyUSB1')

        self.tx_data = []
        self.rx_N = 0
        self.rx_data = []*4

        self.msg = Int16MultiArray()
        for i in range(3):
            self.msg.data.append(0)
        
        self.float_msg = Float32MultiArray()
        for i in range(6):
            self.float_msg.data.append(0)

        self.publisher = self.create_publisher(Int16MultiArray,"IM920_rece_topic",10)
        self.flat_pub = self.create_publisher(Float32MultiArray,"send_topic",10)
        self.subscriber = self.create_subscription(Int16MultiArray,"IM920_send_topic",self.make_send,10)

        self.timer = self.create_timer(0.01,self.interrupt)

    def interrupt(self):
        #self.IM920.write(self.tx_data)

        self.rx_N,self.rx_data = self.IM920.read(16,int)
        print(self.rx_data)
        #self.rx_data = self.IM920.row_read(12)
        #self.msg.data = [self.rx_N,self.rx_data[0],self.rx_data[1]]
        #self.publisher.publish(self.msg)
        #print(self.float_msg.data)
        self.float_msg.data[0] = 0
        self.float_msg.data[1] = float(self.rx_data[0])
        self.float_msg.data[2] = float(self.rx_data[1])
        self.float_msg.data[3] = float(self.rx_data[2])
        self.float_msg.data[4] = 0
        self.float_msg.data[5] = 0
        self.flat_pub.publish(self.float_msg)

    def make_send(self,msg):
        pass

def main(args = None):
    rclpy.init(args=args)

    IM920_node = IM920Node()

    rclpy.spin(IM920_node)

    IM920_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()