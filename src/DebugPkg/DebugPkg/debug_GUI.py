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
        #各ボタンの作成と設置
        self.main_menu()
        self.root.mainloop()

    def main_destroy(self):
        pass
        #try: self.mtr_botton.destroy() 
        #except: pass

    def GUI_reset(self,event):
        self.main_destroy()
        self.main_menu()

    def control(self):  #手動操作用
        print("control")
    
    def main_menu(self):
        self.main_destroy()
        #手動操作
        self.mtr_botton = tk.Button(text="手動操作",font=("メイリオ","20"),command=self.control)
        self.mtr_botton.place(x=20,y=60,width=450,height=200)
        #設定用
        self.setting_button = tk.Button(text="設定",font=("メイリオ","20"),command=self.control)
        self.setting_button.place(x=20,y=10)