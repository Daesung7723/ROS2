import pyautogui
import time
import pyperclip

pyautogui.moveTo(290,20,0.1)
pyautogui.click()
time.sleep(0.5)

pyperclip.copy("google maps vientiane")
pyautogui.hotkey('ctrl','v')
pyautogui.hotkey('enter')
time.sleep(2)

pyautogui.moveTo(375,400,0.2)
pyautogui.click()
time.sleep(4)

pyautogui.moveTo(120,120,2)
pyautogui.doubleClick()
pyperclip.copy("kokkok mart")
pyautogui.hotkey('ctrl','v')
pyautogui.hotkey('enter')
time.sleep(2)

pyautogui.moveTo(100,260,0.2)
pyautogui.click()
