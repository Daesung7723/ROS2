from gtts import gTTS
from playsound import playsound
import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))

text = "hello world. this is my first gTTS try."

tts = gTTS(text=text, lang='en')
tts.save("hi.mp3")
playsound("hi.mp3")