# coding=utf-8
import time

from pydub import AudioSegment
from gtts import gTTS
import pygame
import os

file = os.path.dirname(os.path.abspath(__file__))

SOURCE_gTTS = "./testTest.mp3"
SOURCE_gTTS_wav = "./test.wav"
SOURCE_pico = "./testTest.wav"
SOURCE_MP3 = "talking_file.mp3"
SOURCE_WAV = "talking_file.wav"

mytext = 'Bonjour tout le monde, je testé Google ici avec ce texte!'
language = 'fr-FR'
myobj = gTTS(text=mytext, lang=language, slow=False)
myobj.save(SOURCE_MP3)

text = "\"Bonjour tout le monde, Charles tu suces, Nicolas tu niaises dans la barac, Kate ... j'apprécie ton Mac\""
myText = os.system("pico2wave  -l "+ language +" -w " + SOURCE_pico + " " + text)

print(file+"/"+SOURCE_WAV)
sound = AudioSegment.from_mp3(file+"/"+SOURCE_MP3)
# print(file+SOURCE_WAV)
sound.export(file+"/"+SOURCE_WAV, format="wav")

pygame.mixer.init()
pygame.mixer.music.load(SOURCE_WAV)
# pygame.mixer.music.play(loops=0)
time.sleep(2)
pygame.mixer.music.stop()
# os.system("mpg321 " + SOURCE_gTTS )
# os.system("aplay " + SOURCE_pico )