import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import pyrealsense2 as RS
import math as m

class RSNode(Node):
    def __init__(self):
        super().__init__("RS_node")

        self.pipe = RS.pipeline()
        self.cfg = RS.config()
        self.cfg.enable_stream(RS.stream.pose)
        self.pipe.start(self.cfg)
        self.msg = Float32MultiArray()
        for i in range(3):
            self.msg.data.append(0)

        self.publisher = self.create_publisher(Float32MultiArray,"RS_topic",10)
        self.timer = self.create_timer(0.001,self.interrupt)
    
    def interrupt(self):
        self.frames = self.pipe.wait_for_frames()
        self.pose = self.frames.get_pose_frame()
        if self.pose:
            self.rs_data = self.pose.get_pose_data()
            self.w =  self.rs_data.rotation.w
            self.x = -self.rs_data.rotation.z
            self.y =  self.rs_data.rotation.x
            self.z = -self.rs_data.rotation.y
            self.yaw = m.atan2(2.0 * (self.w*self.z + self.x*self.y), self.w*self.w + self.x*self.x - self.y*self.y - self.z*self.z) * 180.0 / m.pi
            self.msg.data[0] = -1*self.rs_data.translation.x
            self.msg.data[1] = self.rs_data.translation.z
            self.msg.data[2] = -1*self.yaw
            self.publisher.publish(self.msg)
            for i in range(3):
                print("{:.3f}".format(self.msg.data[i]),end=" ")
            print()

def main(args = None):
    rclpy.init(args=args)

    RSnode = RSNode()

    rclpy.spin(RSnode)

    RSnode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
