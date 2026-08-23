import pyautogui

pos = [375,400]
len = 200

pyautogui.click(pos[0], pos[1])
for y in range(20):
    if y%2==0 : pos[1] = pos[1]+len
    else : pos[1] = pos[1]-len
    pyautogui.dragTo(pos[0], pos[1], 0.1)
    
    if y%2==0 : pos[0] = pos[0]+len
    else : pos[0] = pos[0]-len
    pyautogui.dragTo(pos[0], pos[1], 0.1)

    len = len-10

