import tkinter as tk
import threading

class DebugGUI(tk.Tk):
    def __init__(self,window_name="DebugWindow",geometory="1024x600"):
        super().__init__()
        self.title(window_name)
        self.geometry(geometory)

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

    def GUI_reset(self,event):
        self.main_destroy()
        self.main_menu()

    def param_update(self):
        with open("src/DebugPkg/DebugPkg/robot_param.md",mode="w") as param:
            for i in range(15):
                if self.param_entry[i].get() != "":
                    param.write(self.Param[i*2]+"   "+self.param_entry[i].get()+"\n")
                else:
                    param.write(self.Param[i*2]+"   "+self.Param[i*2+1]+"\n")
            for i in range(15,32):
                param.write(self.Param[i*2]+"   "+self.Param[i*2+1]+"\n")
        self.set_param()

    def write_velo(self):
        with open("src/DebugPkg/DebugPkg/robot_param.md",mode="w") as param:
            for i in range(15):
                param.write(self.Param[i*2]+"   "+self.Param[i*2+1]+"\n")
            param.write(self.Param[15*2]+"   "+f"{self.x_vel}"+"\n")
            param.write(self.Param[16*2]+"   "+f"{self.y_vel}"+"\n")
            param.write(self.Param[17*2]+"   "+f"{self.omega}"+"\n")
            for i in range(18,32):
                param.write(self.Param[i*2]+"   "+self.Param[i*2+1]+"\n")   
        with open("src/DebugPkg/DebugPkg/robot_param.md") as param:
            self.Param = param.read().split()
    
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

    def main_menu(self):
        with open("src/DebugPkg/DebugPkg/robot_param.md") as param:
            self.Param = param.read().split()
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
        with open("src/DebugPkg/DebugPkg/robot_param.md") as param:
            self.Param = param.read().split()
            for i in range(15):
                self.param_entry[i] = tk.Entry(font=("メイリオ","20"),width=5)
                self.param_label[i] = tk.Label(text=self.Param[i*2]+"    "+self.Param[i*2+1],font=("メイリオ","20"))
                
                self.param_entry[i].place(x=(i//9)*560+250,y=(i%9)*55+60)
                self.param_label[i].place(x=(i//9)*500+ 10,y=(i%9)*55+60)
    
        self.set_param_button = tk.Button(text="更新",font=("メイリオ","20"),command=self.param_update)
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

    def auto_control(self):
        self.main_destroy()
        self.pos_name_label = [0]*9
        self.pos_label = [0]*9
        with open("src/DebugPkg/DebugPkg/robot_param.md") as param:
            self.Param = param.read().split()
            for i in range(9):
                self.pos_name_label[i] = tk.Label(text=self.Param[36+i*2],font=("メイリオ","20"))
                self.pos_label[i] = tk.Label(text=self.Param[36+i*2+1],font=("メイリオ","20"))
                self.pos_name_label[i].place(x=20,y=80+i*50)
                self.pos_label[i].place(x=100,y=80+i*50)

if __name__ == "__main__":
    gui = DebugGUI()