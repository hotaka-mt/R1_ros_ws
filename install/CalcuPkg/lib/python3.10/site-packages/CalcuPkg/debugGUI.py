import tkinter as tk
import threading

class DebugGUI(tk.Tk):
    def __init__(self,window_name="DebugWindow",geometory="1024x600"):
        super().__init__()
        self.title(window_name)
        self.geometry(geometory)

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
        self.mainloop()

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
            if num != 0: 
                for i in range(3*num):
                    param.write(self.Param[i*2]+"   "+self.Param[i*2+1]+"\n") 
            for i in range(3):
                if self.param_entry[i+3*num].get() != "":
                    param.write(self.Param[i*2+num*6]+"   "+self.param_entry[i+3*num].get()+"\n")
                else:
                    param.write(self.Param[i*2+num*6]+"   "+self.Param[i*2+1+num*6]+"\n")
            if num != 4:
                for i in range(3*(num+1),15):
                    param.write(self.Param[i*2]+"   "+self.Param[i*2+1]+"\n") 

            for i in range(15,len(self.Param)//2):
                param.write(self.Param[i*2]+"   "+self.Param[i*2+1]+"\n")
        self.set_param()

    def write_velo(self):
        with open("src/CalcuPkg/CalcuPkg/robot_param.md",mode="w") as param:
            for i in range(15):
                param.write(self.Param[i*2]+"   "+self.Param[i*2+1]+"\n")
            param.write(self.Param[15*2]+"   "+f"{self.x_vel}"+"\n")
            param.write(self.Param[16*2]+"   "+f"{self.y_vel}"+"\n")
            param.write(self.Param[17*2]+"   "+f"{self.omega}"+"\n")
            for i in range(18,len(self.Param)//2):
                param.write(self.Param[i*2]+"   "+self.Param[i*2+1]+"\n")   
        with open("src/CalcuPkg/CalcuPkg/robot_param.md") as param:
            para = param.read().split()
            if len(para) != 64: return False
            self.Param = para
    
    def update_velo(self,event):
        self.x_vel = self.RL_scale_var.get()
        self.y_vel = self.FB_scale_var.get()
        self.omega = self.Omega_scale_var.get()
        print(f"{self.x_vel}    {self.y_vel}    {self.omega}")
        self.write_velo()

    def reset_velo(self):
        self.x_vel = 0
        self.y_vel = 0
        self.omega = 0
        self.FB_scale_var.set(0)
        self.RL_scale_var.set(0)
        self.Omega_scale_var.set(0)
        print(f"{self.x_vel}    {self.y_vel}    {self.omega}")
        self.write_velo()

    def motion_set0(self):self.motion_set(0)
    def motion_set1(self):self.motion_set(1)
    def motion_set2(self):self.motion_set(2)
    def motion_set3(self):self.motion_set(3)
    def motion_set4(self):self.motion_set(4)
    def motion_set5(self):self.motion_set(5)
    def motion_set6(self):self.motion_set(6)
    def motion_set7(self):self.motion_set(7)
    def motion_set8(self):self.motion_set(8)

    def motion_set(self,num):
        with open("src/CalcuPkg/CalcuPkg/robot_param.md",mode="w") as param:
            for i in range(32):
                param.write(self.Param[i*2]+"   "+self.Param[i*2+1]+"\n")
            param.write(self.Param[64]+f"   {num}\n")

    ###GUI画面
    def main_menu(self):
        with open("src/CalcuPkg/CalcuPkg/robot_param.md") as param:
            para = param.read().split()
            if len(para) != self.param_num: return False
            self.Param = para
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
        with open("src/CalcuPkg/CalcuPkg/robot_param.md") as param:
            para = param.read().split()
            if len(para) != self.param_num: return False
            self.Param = para
            for i in range(15):
                self.param_entry[i] = tk.Entry(font=("メイリオ","20"),width=5)
                self.param_label[i] = tk.Label(text=self.Param[i*2]+"    "+self.Param[i*2+1],font=("メイリオ","20"))
                
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
        self.pos_thread = threading.Thread(target=self.pos_update)
        with open("src/CalcuPkg/CalcuPkg/robot_param.md") as param:
            para = param.read().split()
            if len(para) != self.param_num: return False
            self.Param = para
            for i in range(9):
                self.pos_name_label[i] = tk.Label(text=self.Param[36+i*2],font=("メイリオ","20"))
                self.pos_label[i] = tk.Label(text=self.Param[36+i*2+1],font=("メイリオ","20"))
                self.pos_name_label[i].place(x=20,y=80+i*50)
                self.pos_label[i].place(x=200,y=80+i*50)
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
        self.thread_flag = True
        self.pos_thread.start()

    def pos_update(self):
        while True:
            #print(self.thread_flag)
            if self.thread_flag == False: break
            with open("src/CalcuPkg/CalcuPkg/robot_param.md") as param:
                self.Param = param.read().split()
                #print("hello")
                if len(self.Param) != 0:
                    for i in range(9):
                        try:self.pos_label[i]["text"] = self.Param[36+i*2+1]
                        except:pass

if __name__ == "__main__":
    gui = DebugGUI()