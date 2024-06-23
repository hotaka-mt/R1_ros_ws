from matplotlib.pyplot import plot as plt
import tkinter as tk

class DebugGUI():
    def __init__(self,window_name="DebugWindow",geometory="1024x600"):
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
        self.x_vel = self.FB_scale_var.get()
        self.y_vel = self.RL_scale_var.get()
        self.omega = self.Omega_scale_var.get()
        print(f"{self.x_vel}    {self.y_vel}    {self.omega}")
    
    def reset_velo(self):
        self.x_vel = 0
        self.y_vel = 0
        self.omega = 0
        self.FB_scale_var.set(0)
        self.RL_scale_var.set(0)
        self.Omega_scale_var.set(0)
        print(f"{self.x_vel}    {self.y_vel}    {self.omega}")

    def get_velo(self):
        return self.x_vel,self.y_vel,self.omega

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