import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import UInt8
from std_msgs.msg import Int16MultiArray
import numpy as np
from CalcuPkg import debugGUI as GUI
import tkinter as tk
import threading

class DebugNode(Node):
    def __init__(self):
        super().__init__("debug_node")
        self.serial_send = Float32MultiArray()
        for i in range(8):
            self.serial_send.data.append(0)
        self.auto_msg = UInt8()

        self.rs_data = [0]*3
        self.serial_rece_data = [0]*22
        self.otos_data = [0]*9
        self.im920_data = [0]*3
        self.pos_data = [0]*3

        self.rs_sub = self.create_subscription(Float32MultiArray,"RS_topic",self.get_realsense,10)
        self.serial_sub = self.create_subscription(UInt8MultiArray,"reception_topic",self.get_serial,10)
        self.IM920_sub = self.create_subscription(Int16MultiArray,"IM920_rece_topic",self.get_IM920,10)
        self.pos_sub = self.create_subscription(Float32MultiArray,"pos_topic",self.get_position,10)

        self.serial_pub = self.create_publisher(Float32MultiArray,"send_topic",10)
        self.IM920_pub = self.create_publisher(Float32MultiArray,"IM920_send_topic",10)
        self.auto_pub = self.create_publisher(UInt8,"auto_topic",10)

        self.param_num = 66
        self.Param = [0]*self.param_num
        self.before_Param = [0]*self.param_num
        self.debug_name = ["x位置","y位置","角度",
                        "OTOSx位置","OTOSy位置","OTOS角度",
                        "RSx位置","RSy位置","RS位置",
                        "x速度","y速度","角速度",
                        "OTOSx速度","OTOSy速度","OTOS角速度",
                        "RSx速度","RSy速度","RS角速度",]
        self.debug_param = [0]*18
        self.target_xvelo = 0.0
        self.target_yvelo = 0.0
        self.target_omega = 0.0
        self.machan_data = 0
        self.reload1_bit = 0
        self.reload2_bit = 0
        self.Enkaku_bit = 0
        self.crapult_bit = 0

        self.field = 0
        self.field_color = ["LightPink","SkyBlue"]
        
        self.gui_thread = threading.Thread(target=self.gui_init)
        self.gui_thread.start()

################################Node################################
    def robot_publish(self):
        #print(f"{x} {y} {omega}")
        self.serial_send.data[0] = 0
        self.serial_send.data[1] = float(self.target_xvelo)
        self.serial_send.data[2] = float(self.target_yvelo)
        self.serial_send.data[3] = float(self.target_omega)
        self.serial_send.data[4] = 0
        self.serial_send.data[5] = float(self.machan_data)
        self.serial_send.data[6] = 0
        self.serial_send.data[7] = 0
        print(self.serial_send.data)
        self.serial_pub.publish(self.serial_send)

    def auto_publish(self,num):
        self.auto_msg.data = num | (self.field<<3)
        print(hex(self.auto_msg.data))
        self.auto_pub.publish(self.auto_msg)

    def get_position(self,msg):
        #print(msg.data)
        for i in range(3):
            self.debug_param[i] = msg.data[i]
        for i in range(3):
            self.debug_param[i+9] = msg.data[i+3]
        self.update_robotparam()
    def get_serial(self,msg):
        #print(msg.data)
        for i in range(22):
            self.serial_rece_data[i] = msg.data[i]
        self.debug_param[3] = int(np.array(msg.data[2]<<8 | msg.data[1],dtype=np.int16)) * 0.0003
        self.debug_param[4] = int(np.array(msg.data[4]<<8 | msg.data[3],dtype=np.int16)) * 0.0003
        self.debug_param[5] = int(np.array(msg.data[6]<<8 | msg.data[5],dtype=np.int16)) * 0.0055
        self.debug_param[12] = int(np.array(msg.data[8]<<8 | msg.data[7],dtype=np.int16)) * 0.0015
        self.debug_param[13] = int(np.array(msg.data[10]<<8 | msg.data[9],dtype=np.int16)) * 0.0015
        self.debug_param[14] = int(np.array(msg.data[12]<<8 | msg.data[11],dtype=np.int16)) * 0.061
        #print(self.serial_rece_data)
        self.update_robotparam()
    def get_realsense(self,msg):
        #print(msg.data)
        for i in range(3):
            self.debug_param[i+6] = msg.data[i]
        self.update_robotparam()
    def get_IM920(self,msg):
        #print(msg.data)
        for i in range(3):
            self.im920_data[i] = msg.data[i]
        self.update_robotparam()
    
    def update_robotparam(self):
        for i in range(18):
            try: self.pos_label[i]["text"] = "{:.3f}".format(self.debug_param[i])
            except:pass
            try: self.limit_label[i]["text"] = "ON" if self.serial_rece_data[19]&(0x01<<i) else "OFF"
            except:pass
################################Node################################

################################GUI################################
    def gui_init(self):
        self.gui = tk.Tk()
        self.gui.title("DebugWindow")
        self.gui.geometry("1600x1000")
        self.gui["bg"] = self.field_color[self.field]

        self.param_num = 66

        #リセットボタンを作成
        self.home_button = tk.Button(text = "ホームに戻る",font = ("MSゴシック","20"))
        self.home_button.bind('<Button-1>',self.GUI_reset)
        self.home_button.place(x=1225,y=10)
        self.field_button = tk.Button(text='フィールド変更',font = ("MSゴシック","20"))
        self.field_button.bind('<Button-1>',self.field_change)
        self.field_button.place(x=800,y=10)
        #各ボタンの作成と設置
        self.main_menu()
        self.gui.mainloop()

    def main_destroy(self):
        self.thread_flag = False
        try: self.manual_botton.destroy() 
        except: pass
        try: self.auto_botton.destroy() 
        except: pass
        try: self.FB_scale.destroy()
        except: pass
        try: self.RL_scale.destroy()
        except: pass
        try: self.Omega_scale.destroy()
        except: pass
        try: self.reset_button.destroy()
        except: pass
        for i in range(18):
            try: self.pos_name_label[i].destroy()
            except:pass
            try: self.pos_label[i].destroy()
            except:pass
            try: self.limit_name_label[i].destroy()
            except:pass
            try: self.limit_label[i].destroy()
            except:pass
        for i in range(8):
            try: self.motion_button[i].destroy()
            except: pass
        try: self.motion_reset.destroy()
        except: pass
        try: self.Enkaku_button.destroy()
        except: pass
        try: self.crapult_button.destroy()
        except: pass
        try: self.reload1_button.destroy()
        except: pass
        try: self.reload2_button.destroy()
        except: pass

    def GUI_reset(self,event):
        self.main_destroy()
        self.main_menu()

    def field_change(self,event):
        self.field = (self.field+1)%2
        self.gui["bg"] = self.field_color[self.field]
    
    def update_velo(self,event):
        self.target_xvelo = self.RL_scale_var.get()
        self.target_yvelo = self.FB_scale_var.get()
        self.target_omega = self.Omega_scale_var.get()
        #print(f"{self.target_xvelo}    {self.target_yvelo}    {self.target_omega}")
        self.robot_publish()

    def reset_velo(self):
        self.target_xvelo = 0
        self.target_yvelo = 0
        self.target_omega = 0
        self.FB_scale_var.set(0)
        self.RL_scale_var.set(0)
        self.Omega_scale_var.set(0)
        #print(f"{self.target_xvelo}    {self.target_yvelo}    {self.target_omega}")
        self.robot_publish()

    def update_machan(self,num):
        if num==1:
            self.reload1_bit = 1
            self.reload2_bit = 0
        if num==2:
            self.reload1_bit = 0
            self.reload2_bit = 1
        if num == 3: self.Enkaku_bit = not(self.Enkaku_bit)
        if num == 4: self.crapult_bit = not(self.crapult_bit)

        self.machan_data = (self.crapult_bit<<3) | (self.Enkaku_bit<<2) | (self.reload2_bit<<1) | (self.reload1_bit<<0)
        self.robot_publish()

    def reload1_machan(self): self.update_machan(1)
    def reload2_machan(self): self.update_machan(2)
    def Enkaku_machan(self): self.update_machan(3)
    def crapult_machan(self): self.update_machan(4)

    def motion_set0(self):self.auto_publish(0x00)
    def motion_set1(self):self.auto_publish(0x11)
    def motion_set2(self):self.auto_publish(0x12)
    def motion_set3(self):self.auto_publish(0x23)
    def motion_set4(self):self.auto_publish(0x04)
    def motion_set5(self):self.auto_publish(5)
    def motion_set6(self):self.auto_publish(6)
    def motion_set7(self):self.auto_publish(7)
    def motion_set8(self):self.auto_publish(8)

    ###GUI画面
    def main_menu(self):
        self.main_destroy()
        #手動操作
        self.manual_botton = tk.Button(text="手動操作",font=("メイリオ","20"),command=self.manual_control)
        self.manual_botton.place(x=20,y=100,width=760,height=300)
        #自動操作
        self.auto_botton = tk.Button(text="自動操作",font=("メイリオ","20"),command=self.auto_control)
        self.auto_botton.place(x=820,y=100,width=760,height=300)

    def manual_control(self):
        self.main_destroy()
        self.reset_button = tk.Button(text="リセット",font=("メイリオ","20"),command=self.reset_velo)
        self.FB_scale_var = tk.IntVar()
        self.RL_scale_var = tk.IntVar()
        self.Omega_scale_var = tk.IntVar()

        self.Enkaku_button = tk.Button(text="遠隔探索機発射",font=("メイリオ","20"),command=self.Enkaku_machan)
        self.crapult_button = tk.Button(text="鶴パルト発射",font=("メイリオ","20"),command=self.crapult_machan)
        self.reload1_button = tk.Button(text="お迎え",font=("メイリオ","20"),command=self.reload1_machan)
        self.reload2_button = tk.Button(text="リロード",font=("メイリオ","20"),command=self.reload2_machan)

        self.Enkaku_button.place(x=500,y=150)
        self.crapult_button.place(x=1000,y=150)
        self.reload1_button.place(x=500,y=250)
        self.reload2_button.place(x=1000,y=250)

        self.FB_scale    = tk.Scale(variable=self.FB_scale_var   ,orient=tk.VERTICAL  ,length=900,width=100,sliderlength=40,from_=  1500,to= -1500,tickinterval=500,command=self.update_velo)
        self.RL_scale    = tk.Scale(variable=self.RL_scale_var   ,orient=tk.HORIZONTAL,length=900,width=100,sliderlength=40,from_= -1500,to=  1500,tickinterval=500,command=self.update_velo)
        self.Omega_scale = tk.Scale(variable=self.Omega_scale_var,orient=tk.HORIZONTAL,length=600,width=100,sliderlength=40,from_= -180 ,to=  180, tickinterval=45 ,command=self.update_velo)
        self.reset_button.place(x=1200,y=850)
        self.FB_scale.place(x=50,y=60)
        self.RL_scale.place(x=500,y=400)
        self.Omega_scale.place(x=650,y=600)

    def auto_control(self):
        self.main_destroy()
        self.pos_name_label = [0]*18
        self.pos_label = [0]*18
        self.limit_name_label = [0]*9
        self.limit_label = [0]*9
        for i in range(18):
            self.pos_name_label[i] = tk.Label(text=self.debug_name[i],font=("メイリオ","17"))
            self.pos_label[i] = tk.Label(text=self.debug_param[i],font=("メイリオ","17"))
            self.pos_name_label[i].place(x= 20 if i<9 else 500,y=160+(i%9)*80)
            self.pos_label[i].place(x= 300 if i<9 else 780,y=160+(i%9)*80)
        for i in range(8):
            self.limit_name_label[i] = tk.Label(text=f"リミット{i+1}",font=("メイリオ","17"))
            self.limit_label[i] = tk.Label(text="OFF",font=("メイリオ","17"))
            self.limit_name_label[i].place(x=970,y=160+i*80)
            self.limit_label[i].place(x=1225,y=160+i*80)
        self.motion_button = [0]*8
        self.motion_button[0] = tk.Button(text="E100",font=("メイリオ","17"),command=self.motion_set1)
        self.motion_button[1] = tk.Button(text="E000",font=("メイリオ","17"),command=self.motion_set2)
        self.motion_button[2] = tk.Button(text="K000",font=("メイリオ","17"),command=self.motion_set3)
        self.motion_button[3] = tk.Button(text="Stay",font=("メイリオ","17"),command=self.motion_set4)
        self.motion_button[4] = tk.Button(text="動作5",font=("メイリオ","17"),command=self.motion_set5)
        self.motion_button[5] = tk.Button(text="動作6",font=("メイリオ","17"),command=self.motion_set6)
        self.motion_button[6] = tk.Button(text="動作7",font=("メイリオ","17"),command=self.motion_set7)
        self.motion_button[7] = tk.Button(text="動作8",font=("メイリオ","17"),command=self.motion_set8)
        self.motion_reset = tk.Button(text="Start",font=("メイリオ","17"),command=self.motion_set0)
        for i in range(8):
            self.motion_button[i].place(x=1350,y=160+i*80)
        self.motion_reset.place(x=1350,y=800)
################################GUI################################

def main(args = None):
    rclpy.init(args=args)

    debug_node = DebugNode()
    rclpy.spin(debug_node)
    debug_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
