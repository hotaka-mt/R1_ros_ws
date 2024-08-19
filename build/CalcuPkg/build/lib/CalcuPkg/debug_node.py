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
        self.serial_rece_data = [0]*10
        self.otos_data = [0]*3
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
        self.param_name = ["M1Pゲイン","M1Iゲイン","M1Dゲイン",
                        "M1Pゲイン","M1Iゲイン","M1Dゲイン",
                        "M1Pゲイン","M1Iゲイン","M1Dゲイン",
                        "M1Pゲイン","M1Iゲイン","M1Dゲイン",
                        "タイヤ径","旋回直径","ロリコン分解能"]
        self.debug_name = ["x位置","y位置","角度",
                        "RSx位置","RSy位置","RS角度",
                        "OTOSx位置","OTOSy位置","OTOS角度",]
        self.robot_param = []
        with open("src/CalcuPkg/CalcuPkg/robot_param.md") as param:
            Param = param.read().split()
            for i in range(len(Param)):
                self.robot_param.append(float(Param[i]))
        self.debug_param = [0,0,0,0,0,0,0,0,0]
        self.target_xvelo = 0.0
        self.target_yvelo = 0.0
        self.target_omega = 0.0
        
        self.gui_thread = threading.Thread(target=self.gui_init)
        self.gui_thread.start()

################################Node################################
    def mecanum_publish(self,x,y,omega):
        #print(f"{x} {y} {omega}")
        self.serial_send.data[0] = 0
        self.serial_send.data[1] = float(x)
        self.serial_send.data[2] = float(y)
        self.serial_send.data[3] = float(omega)
        self.serial_send.data[4] = 0
        self.serial_send.data[5] = 0
        if x==0 and y==0 and omega==0:
            self.serial_send.data[5] = 0x01
        self.serial_send.data[6] = 0
        self.serial_send.data[7] = 0
        self.serial_pub.publish(self.serial_send)

    def param_publish(self,num,P,I,D):
        self.serial_send.data[0] = float(num)
        self.serial_send.data[1] = float(P)
        self.serial_send.data[2] = float(I)
        self.serial_send.data[3] = float(D)
        self.serial_send.data[4] = 0
        self.serial_send.data[5] = 0
        self.serial_send.data[6] = 0
        self.serial_send.data[7] = 0
        #print(self.serial_send.data)
        self.serial_pub.publish(self.serial_send)

    def auto_publish(self,num):
        self.auto_msg.data = num
        self.auto_pub.publish(self.auto_msg)

    def get_position(self,msg):
        #print(msg.data)
        for i in range(3):
            self.debug_param[i] = msg.data[i]
        self.update_robotparam()
    def get_serial(self,msg):
        #print(msg.data)
        for i in range(10):
            self.serial_rece_data[i] = msg.data[i]
        self.debug_param[6] = int(np.array(msg.data[1]<<8 | msg.data[0],dtype=np.int16)) * 0.0003
        self.debug_param[7] = int(np.array(msg.data[3]<<8 | msg.data[2],dtype=np.int16)) * 0.0003
        self.debug_param[8] = int(np.array(msg.data[5]<<8 | msg.data[4],dtype=np.int16)) * 0.0055
        #print(self.serial_rece_data)
        self.update_robotparam()
    def get_realsense(self,msg):
        #print(msg.data)
        for i in range(3):
            self.debug_param[i+3] = msg.data[i]
        self.update_robotparam()
    def get_IM920(self,msg):
        #print(msg.data)
        for i in range(3):
            self.im920_data[i] = msg.data[i]
        self.update_robotparam()
    
    def update_robotparam(self):
        for i in range(9):
            try: self.pos_label[i]["text"] = "{:.3f}".format(self.debug_param[i])
            except:pass
            try: self.limit_label[i]["text"] = "ON" if self.serial_rece_data[7]&(0x01<<i) else "OFF"
            except:pass
################################Node################################

################################GUI################################
    def gui_init(self):
        self.gui = tk.Tk()
        self.gui.title("DebugWindow")
        self.gui.geometry("1024x600")

        self.param_num = 66

        #リセットボタンを作成
        self.home_button = tk.Button(text = "ホームに戻る",font = ("MSゴシック","20"))
        self.home_button.bind('<Button-1>',self.GUI_reset)
        self.home_button.place(x=830,y=10)
        #設定ボタン作成
        self.setting_button = tk.Button(text="設定",font=("メイリオ","20"),command=self.set_param)
        self.setting_button.place(x=20,y=10)
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
        for i in range(15):
            try: self.param_entry[i].destroy()
            except: pass
            try: self.param_label[i].destroy()
            except: pass
            try: self.param_button[i].destroy()
            except: pass
        try: self.set_param_button.destroy()
        except: pass
        for i in range(9):
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

    def GUI_reset(self,event):
        self.main_destroy()
        self.main_menu()

    def param_update1(self):self.param_update(0)
    def param_update2(self):self.param_update(1)
    def param_update3(self):self.param_update(2)
    def param_update4(self):self.param_update(3)
    def param_update5(self):self.param_update(4)

    def param_update(self,num):
        with open("src/CalcuPkg/CalcuPkg/robot_param.md",mode="w") as param:
            if num!=0:
                for i in range(0,num*3): param.write(f"{self.robot_param[i]}\n")
            for i in range(num*3,num*3+3): 
                if self.param_entry[i].get() != "":
                    param.write(self.param_entry[i].get()+"\n")
                else:
                    param.write(f"{self.robot_param[i]}\n")
            if num!=4:
                for i in range(num*3+3,15): param.write(f"{self.robot_param[i]}\n")
        
        for i in range(3):
            if self.param_entry[num*3+i].get() != "":
                self.robot_param[num*3+i] = float(self.param_entry[num*3+i].get())
                
        self.param_publish(num+1,self.robot_param[num*3],self.robot_param[num*3+1],self.robot_param[num*3+2])
        print(self.robot_param)
        self.set_param()
    
    def update_velo(self,event):
        self.target_xvelo = self.RL_scale_var.get()
        self.target_yvelo = self.FB_scale_var.get()
        self.target_omega = self.Omega_scale_var.get()
        print(f"{self.target_xvelo}    {self.target_yvelo}    {self.target_omega}")
        self.mecanum_publish(self.target_xvelo,self.target_yvelo,self.target_omega)

    def reset_velo(self):
        self.target_xvelo = 0
        self.target_yvelo = 0
        self.target_omega = 0
        self.FB_scale_var.set(0)
        self.RL_scale_var.set(0)
        self.Omega_scale_var.set(0)
        print(f"{self.target_xvelo}    {self.target_yvelo}    {self.target_omega}")
        self.mecanum_publish(self.target_xvelo,self.target_yvelo,self.target_omega)

    def motion_set0(self):self.auto_publish(0)
    def motion_set1(self):self.auto_publish(1)
    def motion_set2(self):self.auto_publish(2)
    def motion_set3(self):self.auto_publish(3)
    def motion_set4(self):self.auto_publish(4)
    def motion_set5(self):self.auto_publish(5)
    def motion_set6(self):self.auto_publish(6)
    def motion_set7(self):self.auto_publish(7)
    def motion_set8(self):self.auto_publish(8)

    ###GUI画面
    def main_menu(self):
        self.main_destroy()
        #手動操作
        self.manual_botton = tk.Button(text="手動操作",font=("メイリオ","20"),command=self.manual_control)
        self.manual_botton.place(x=20,y=60,width=450,height=200)
        #自動操作
        self.auto_botton = tk.Button(text="自動操作",font=("メイリオ","20"),command=self.auto_control)
        self.auto_botton.place(x=500,y=60,width=450,height=200)
    
    def set_param(self):
        self.main_destroy()
        self.param_entry = [0]*15
        self.param_label = [0]*15
        self.param_button = [0]*5
        
        for i in range(15):
            self.param_entry[i] = tk.Entry(font=("メイリオ","20"),width=5)
            self.param_label[i] = tk.Label(text=self.param_name[i]+f"    {self.robot_param[i]}",font=("メイリオ","20"))
            
            self.param_entry[i].place(x=(i//9)*560+250,y=(i%9)*55+60)
            self.param_label[i].place(x=(i//9)*500+ 10,y=(i%9)*55+60)

        self.param_button[0] = tk.Button(text="更新",font=("メイリオ","20"),command=self.param_update1)
        self.param_button[1] = tk.Button(text="更新",font=("メイリオ","20"),command=self.param_update2)
        self.param_button[2] = tk.Button(text="更新",font=("メイリオ","20"),command=self.param_update3)
        self.param_button[3] = tk.Button(text="更新",font=("メイリオ","20"),command=self.param_update4)
        self.param_button[4] = tk.Button(text="更新",font=("メイリオ","20"),command=self.param_update5)
        for i in range(5):
            self.param_button[i].place(x=(i//3)*560+350,y=(i%3)*165+170)

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

    def auto_control(self):
        self.main_destroy()
        self.pos_name_label = [0]*9
        self.pos_label = [0]*9
        self.limit_name_label = [0]*9
        self.limit_label = [0]*9
        for i in range(9):
            self.pos_name_label[i] = tk.Label(text=self.debug_name[i],font=("メイリオ","20"))
            self.pos_label[i] = tk.Label(text=self.debug_param[i],font=("メイリオ","20"))
            self.limit_name_label[i] = tk.Label(text=f"リミット{i+1}",font=("メイリオ","20"))
            self.limit_label[i] = tk.Label(text="OFF",font=("メイリオ","20"))
            self.pos_name_label[i].place(x=20,y=80+i*50)
            self.pos_label[i].place(x=200,y=80+i*50)
            self.limit_name_label[i].place(x=330,y=80+i*50)
            self.limit_label[i].place(x=470,y=80+i*50)
        self.motion_button = [0]*8
        self.motion_button[0] = tk.Button(text="動作1",font=("メイリオ","20"),command=self.motion_set1)
        self.motion_button[1] = tk.Button(text="動作2",font=("メイリオ","20"),command=self.motion_set2)
        self.motion_button[2] = tk.Button(text="動作3",font=("メイリオ","20"),command=self.motion_set3)
        self.motion_button[3] = tk.Button(text="動作4",font=("メイリオ","20"),command=self.motion_set4)
        self.motion_button[4] = tk.Button(text="動作5",font=("メイリオ","20"),command=self.motion_set5)
        self.motion_button[5] = tk.Button(text="動作6",font=("メイリオ","20"),command=self.motion_set6)
        self.motion_button[6] = tk.Button(text="動作7",font=("メイリオ","20"),command=self.motion_set7)
        self.motion_button[7] = tk.Button(text="動作8",font=("メイリオ","20"),command=self.motion_set8)
        self.motion_reset = tk.Button(text="初期位置",font=("メイリオ","20"),command=self.motion_set0)
        for i in range(8):
            self.motion_button[i].place(x=850,y=80+i*50)
        self.motion_reset.place(x=850,y=480)
################################GUI################################

def main(args = None):
    rclpy.init(args=args)

    debug_node = DebugNode()
    rclpy.spin(debug_node)
    debug_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
