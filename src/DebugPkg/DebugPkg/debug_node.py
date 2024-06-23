import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int16MultiArray
import tkinter as tk

class DebugNode(Node):
    def __init__(self,window_name="DebugWindow",geometory="1024x600"):
        super().__init__("debug_node")

        self.rs_data = [0]*3
        self.serial = Float32MultiArray()
        for i in range(5):
            self.serial.data.append(0)
        self.im920_data = [0]*3
        self.pos_data = [0]*4

        self.rs_sub = self.create_subscription(Float32MultiArray,"RS_topic",self.get_realsense,10)
        self.serial_sub = self.create_subscription(Float32MultiArray,"reception_topic",self.get_serial,10)
        self.IM920_sub = self.create_subscription(Int16MultiArray,"IM920_rece_topic",self.get_IM920,10)
        self.pos_sub = self.create_subscription(Float32MultiArray,"pos_topic",self.get_position,10)

        self.serial_pub = self.create_publisher(Float32MultiArray,"send_topic",10)
        self.IM920_pub = self.create_publisher(Float32MultiArray,"IM920_send_topic",10)
        
        #ウィンドウ設定
        self.root = tk.Tk()
        self.root.title(window_name)
        self.root.geometry(geometory)
        #リセットボタンを作成
        self.reset_button = tk.Button(text = "ホームに戻る",font = ("MSゴシック","20"))
        self.reset_button.bind('<Button-1>',self.GUI_reset)
        self.reset_button.place(x=830,y=10)

        self.x_vel = 0
        self.y_vel = 0
        self.omega = 0

        #各ボタンの作成と設置
        self.main_menu()
        self.root.mainloop()


    def main_destroy(self):
        try: self.manual_botton.destroy() 
        except: pass

    def GUI_reset(self,event):
        self.main_destroy()
        self.main_menu()

    def control(self):  #手動操作用
        print("control")
    
    def update_velo(self,event):
        self.x_vel = self.RL_scale_var.get()
        self.y_vel = self.FB_scale_var.get()
        self.omega = self.Omega_scale_var.get()
        print(f"{self.x_vel}    {self.y_vel}    {self.omega}")
        self.serialpublish(self.x_vel,self.y_vel,self.omega)

    def reset_velo(self):
        self.x_vel = 0
        self.y_vel = 0
        self.omega = 0
        self.FB_scale_var.set(0)
        self.RL_scale_var.set(0)
        self.Omega_scale_var.set(0)
        print(f"{self.x_vel}    {self.y_vel}    {self.omega}")
        self.serialpublish(self.x_vel,self.y_vel,self.omega)

    def main_menu(self):
        self.main_destroy()
        #手動操作
        self.manual_botton = tk.Button(text="手動操作",font=("メイリオ","20"),command=self.manual_control)
        self.manual_botton.place(x=20,y=60,width=450,height=200)
        #設定用
        self.setting_button = tk.Button(text="設定",font=("メイリオ","20"),command=self.control)
        self.setting_button.place(x=20,y=10)

    def manual_control(self):
        self.main_destroy()
        self.reset_button = tk.Button(text="リセット",font=("メイリオ","20"),command=self.reset_velo)
        self.FB_scale_var = tk.IntVar()
        self.RL_scale_var = tk.IntVar()
        self.Omega_scale_var = tk.IntVar()
        self.FB_scale    = tk.Scale(variable=self.FB_scale_var   ,orient=tk.VERTICAL  ,length=500,width=50,sliderlength=40,from_=  1500,to= -1500,tickinterval=500,command=self.update_velo)
        self.RL_scale    = tk.Scale(variable=self.RL_scale_var   ,orient=tk.HORIZONTAL,length=500,width=50,sliderlength=40,from_= -1500,to=  1500,tickinterval=500,command=self.update_velo)
        self.Omega_scale = tk.Scale(variable=self.Omega_scale_var,orient=tk.HORIZONTAL,length=300,width=50,sliderlength=40,from_= -180 ,to=  180, tickinterval=45 ,command=self.update_velo)
        self.reset_button.place(x=800,y=500)
        self.FB_scale.place(x=50,y=60)
        self.RL_scale.place(x=300,y=270)
        self.Omega_scale.place(x=400,y=400)

    def serialpublish(self,x,y,omega):
        self.serial.data[0] = float(x)
        self.serial.data[1] = float(y)
        self.serial.data[2] = float(omega)
        self.serial.data[3] = 0
        self.serial.data[4] = 0
        self.serial_pub.publish(self.serial)

    def get_realsense(self,msg):
        for i in range(3):
            self.rs_data[i] = msg.data[i]
    def get_serial(self,msg):
        for i in range(4):
            self.serial_data[i] = msg.data[i]
    def get_IM920(self,msg):
        for i in range(3):
            self.im920_data[i] = msg.data[i]
    def get_position(self,msg):
        for i in range(3):
            self.pos_data[i] = msg.data[i]

def main(args = None):
    rclpy.init(args=args)

    debug_node = DebugNode()

    rclpy.spin(debug_node)
    debug_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
