import threading
import pyautogui
import time

def thread_auto_draw():
    threading.Timer(15, thread_auto_draw).start()
    pos = [2150,-50]
    len = 200

    pyautogui.click(2210,-268)
    time.sleep(0.5)
    pyautogui.click(2210,-268)
    pyautogui.click(pos[0], pos[1])
    for y in range(20):
        if y%2==0 : pos[1] = pos[1]+len
        else : pos[1] = pos[1]-len
        pyautogui.dragTo(pos[0], pos[1], 0.1)
        
        if y%2==0 : pos[0] = pos[0]+len
        else : pos[0] = pos[0]-len
        pyautogui.dragTo(pos[0], pos[1], 0.1)

        len = len-10

    time.sleep(3)
    pyautogui.click(2100,-270)
    pyautogui.click(2120,-80)
    pyautogui.dragTo(2400,200)
    pyautogui.hotkey('del')

thread_auto_draw()