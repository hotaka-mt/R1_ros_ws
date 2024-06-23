import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import Int16MultiArray
import tkinter as tk

class DebugNode(Node):
    def __init__(self,window_name="DebugWindow",geometory="1024x600"):
        super().__init__("debug_node")
        self.serial_send = Float32MultiArray()
        for i in range(6):
            self.serial_send.data.append(0)

        self.rs_data = [0]*3
        self.serial_rece_data = [0]*38
        self.im920_data = [0]*3
        self.pos_data = [0]*4

        self.rs_sub = self.create_subscription(Float32MultiArray,"RS_topic",self.get_realsense,10)
        self.serial_sub = self.create_subscription(UInt8MultiArray,"reception_topic",self.get_serial,10)
        self.IM920_sub = self.create_subscription(Int16MultiArray,"IM920_rece_topic",self.get_IM920,10)
        self.pos_sub = self.create_subscription(Float32MultiArray,"pos_topic",self.get_position,10)

        self.serial_pub = self.create_publisher(Float32MultiArray,"send_topic",10)
        self.IM920_pub = self.create_publisher(Float32MultiArray,"IM920_send_topic",10)

        self.x_vel = 0
        self.y_vel = 0
        self.omega = 0
        
        #ウィンドウ設定
        self.root = tk.Tk()
        self.root.title(window_name)
        self.root.geometry(geometory)
        #リセットボタンを作成
        self.home_button = tk.Button(text = "ホームに戻る",font = ("MSゴシック","20"))
        self.home_button.bind('<Button-1>',self.GUI_reset)
        self.home_button.place(x=830,y=10)
        #設定ボタン作成
        self.setting_button = tk.Button(text="設定",font=("メイリオ","20"),command=self.set_param)
        self.setting_button.place(x=20,y=10)
        #各ボタンの作成と設置
        self.main_menu()
        self.root.mainloop()

    def main_destroy(self):
        try: self.manual_botton.destroy() 
        except: pass
        try: self.FB_scale.destroy()
        except: pass
        try: self.RL_scale.destroy()
        except: pass
        try: self.Omega_scale.destroy()
        except: pass
        try: self.reset_button.destroy()
        except: pass
        for i in range(15):
            try: self.param_entry[i].destroy()
            except: pass
            try: self.param_label[i].destroy()
            except: pass
            try: self.param_button[i].destroy()
            except: pass
        try: self.set_param_button.destroy()
        except: pass

    def GUI_reset(self,event):
        self.main_destroy()
        self.main_menu()

    def M1_update(self): self.param_publish(0)
    def M2_update(self): self.param_publish(1)
    def M3_update(self): self.param_publish(2)
    def M4_update(self): self.param_publish(3)
    def other_update(self): self.param_publish(4)

    def param_update(self):
        with open("src/DebugPkg/DebugPkg/robot_param.md",mode="w") as param:
            for i in range(15):
                if self.param_entry[i].get() != "":
                    param.write(self.Param[i*2]+"   "+self.param_entry[i].get()+"\n")
                else:
                    param.write(self.Param[i*2]+"   "+self.Param[i*2+1]+"\n")
        self.set_param()
    
    def update_velo(self,event):
        self.x_vel = self.RL_scale_var.get()
        self.y_vel = self.FB_scale_var.get()
        self.omega = self.Omega_scale_var.get()
        print(f"{self.x_vel}    {self.y_vel}    {self.omega}")
        self.mecanum_publish(self.x_vel,self.y_vel,self.omega)

    def reset_velo(self):
        self.x_vel = 0
        self.y_vel = 0
        self.omega = 0
        self.FB_scale_var.set(0)
        self.RL_scale_var.set(0)
        self.Omega_scale_var.set(0)
        print(f"{self.x_vel}    {self.y_vel}    {self.omega}")
        self.mecanum_publish(self.x_vel,self.y_vel,self.omega)

    def main_menu(self):
        self.main_destroy()
        #手動操作
        self.manual_botton = tk.Button(text="手動操作",font=("メイリオ","20"),command=self.manual_control)
        self.manual_botton.place(x=20,y=60,width=450,height=200)
    
    def set_param(self):
        self.main_destroy()
        self.param_entry = [0]*15
        self.param_label = [0]*15
        self.param_button = [0]*5
        with open("src/DebugPkg/DebugPkg/robot_param.md") as param:
            self.Param = param.read().split()
            for i in range(15):
                self.param_entry[i] = tk.Entry(font=("メイリオ","20"),width=5)
                self.param_label[i] = tk.Label(text=self.Param[i*2]+"    "+self.Param[i*2+1],font=("メイリオ","20"))
                
                self.param_entry[i].place(x=(i//9)*560+250,y=(i%9)*55+60)
                self.param_label[i].place(x=(i//9)*500+ 10,y=(i%9)*55+60)
    
        self.param_button[0] = tk.Button(text="反映",font=("メイリオ","20"),command=self.M1_update)
        self.param_button[1] = tk.Button(text="反映",font=("メイリオ","20"),command=self.M2_update)
        self.param_button[2] = tk.Button(text="反映",font=("メイリオ","20"),command=self.M3_update)
        self.param_button[3] = tk.Button(text="反映",font=("メイリオ","20"),command=self.M4_update)
        self.param_button[4] = tk.Button(text="反映",font=("メイリオ","20"),command=self.other_update)
        self.set_param_button = tk.Button(text="更新",font=("メイリオ","20"),command=self.param_update)
        for i in range(5):
            self.param_button[i].place(x=(i//3)*560+350,y=(i%3)*165+170,width=80,height=40)
        self.set_param_button.place(x=800,y=450,height=80,width=80)

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

    def mecanum_publish(self,x,y,omega):
        self.serial_send.data[0] = 0
        self.serial_send.data[1] = float(x)
        self.serial_send.data[2] = float(y)
        self.serial_send.data[3] = float(omega)
        self.serial_send.data[4] = 0
        self.serial_send.data[5] = 0
        self.serial_pub.publish(self.serial_send)

    def param_publish(self,num):
        self.serial_send.data[0] = float(num+1)
        self.serial_send.data[1] = float(self.Param[(num*3)*2+1])
        self.serial_send.data[2] = float(self.Param[(num*3)*2+1+2])
        self.serial_send.data[3] = float(self.Param[(num*3)*2+1+4])
        self.serial_send.data[4] = 0
        self.serial_send.data[5] = 0
        self.serial_pub.publish(self.serial_send)

    def get_realsense(self,msg):
        #print(msg.data)
        for i in range(3):
            self.rs_data[i] = msg.data[i]
    def get_serial(self,msg):
        #print(msg.data)
        for i in range(38):
            self.serial_rece_data[i] = msg.data[i]
    def get_IM920(self,msg):
        #print(msg.data)
        for i in range(3):
            self.im920_data[i] = msg.data[i]
    def get_position(self,msg):
        #print(msg.data)
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
