import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
from SerialPkg import IM920sL

class IM920Node(Node):
    def __init__(self):
        super().__init__ ('IM920_node')
        self.IM920 = IM920sL.IM920('COM3')

        self.tx_data = []
        self.rx_N = 0
        self.rx_data = []

        self.msg = Int16MultiArray()
        for i in range(3):
            self.msg.data.append(0)

        self.publisher = self.create_publisher(Int16MultiArray,"IM920_rece_topic",10)
        self.subscriber = self.create_subscription(Int16MultiArray,"IM920_send_topic",self.make_send,10)

        self.timer = self.create_timer(0.01,self.interrupt)

    def interrupt(self):
        self.IM920.write(self.tx_data)

        self.rx_N,self.rx_data = self.IM920.read(8,int)
        self.msg.data = [self.rx_N,self.rx_data[0],self.rx_data[1]]
        self.publisher.publish(self.msg)

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