import tkinter
import tkinter.font
import os

os.chdir(os.path.dirname(os.path.abspath(__file__)))

def btn_clk():
    global flag
    print("Click")
    flag = not flag

win = tkinter.Tk()
win.title("Test title")
win.geometry("500x500")
# win.resizable(False,False)

set_font = tkinter.font.Font(size=40)
label = tkinter.Label(win, text='', font=set_font)
label.pack()
label2= tkinter.Label(win, text='', font=set_font)
label2.pack()

img = tkinter.PhotoImage(file='Python_image.png')
image = tkinter.Label(win, image=img)
image.pack(expand=1, anchor='center')

button = tkinter.Button(win, text='button', padx=30, pady=15, fg='white', bg='black',font=20, command=btn_clk)
button.pack()

cnt = 0
flag = False

def run_1sec():
    global cnt
    
    cnt = cnt+1
    label.config(text=str(cnt))
    win.after(1000, run_1sec)

def run_100ms():
    global flag
    if flag :
        label2.config(text='Result',fg='red' )
    else :
        label2.config(text='Result',fg='blue' )
    win.after(100, run_100ms )

run_1sec()
run_100ms()
win.mainloop()
