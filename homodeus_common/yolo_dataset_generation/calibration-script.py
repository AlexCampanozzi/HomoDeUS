import cv2
import numpy as np
from tkinter import *
# import ttk
from tkinter import ttk
from PIL import Image, ImageTk
import os


CALIBRATION_FILE = "./calibration/pringles.jpg"


def load_image(filename):
    image = cv2.imread(filename)
    image = cv2.resize(image, (640, 480))
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    return image

def format_image_for_tk(image, hsv=True):
    if hsv:
        new_image = cv2.cvtColor(image, cv2.COLOR_HSV2BGR)
        blue, green, red = cv2.split(new_image)
        img = cv2.merge((red, green, blue))
    else:
        img = image

    im = Image.fromarray(img)
    imgtk = ImageTk.PhotoImage(image=im)
    return imgtk

master = Tk()
img_global = load_image(CALIBRATION_FILE)
img_tk_global = format_image_for_tk(img_global)

def filter_image(image, lh, ls, lv, uh, us, uv):
    l_green = np.array([lh, ls, lv])
    u_green = np.array([uh, us, uv])

    mask = cv2.inRange(image, l_green, u_green)
    return mask

def on_slider_change(event):
    global img_global, img_tk_global
    lh = low_green_h.get()
    ls = low_green_s.get()
    lv = low_green_v.get()

    uh = upper_green_h.get()
    us = upper_green_s.get()
    uv = upper_green_v.get()

    tp_img = filter_image(img_global, lh, ls, lv, uh, us, uv)
    img_tk_global = format_image_for_tk(tp_img, hsv=False)
    image_container.configure(image=img_tk_global)
    image_container.image = img_tk_global

image_container = Label(master, image=img_tk_global)
image_container.pack()

Label(master, text="Filter's lower limit").pack()
low_green_h = Scale(master, from_=0, to=255, orient=HORIZONTAL, length=600, label="Hue:", command=on_slider_change)
low_green_s = Scale(master, from_=0, to=255, orient=HORIZONTAL, length=600, label="Saturation:", command=on_slider_change)
low_green_v = Scale(master, from_=0, to=255, orient=HORIZONTAL, length=600, label="Value:", command=on_slider_change)

low_green_h.pack()
low_green_s.pack()
low_green_v.pack()

separator = ttk.Separator(master, orient='horizontal')
separator.pack(fill='x')

Label(master, text="Filter's upper limit").pack()
upper_green_h = Scale(master, from_=0, to=255, orient=HORIZONTAL, length=600, label="Hue:", command=on_slider_change)
upper_green_s = Scale(master, from_=0, to=255, orient=HORIZONTAL, length=600, label="Saturation:", command=on_slider_change)
upper_green_v = Scale(master, from_=0, to=255, orient=HORIZONTAL, length=600, label="Value:", command=on_slider_change)

upper_green_h.pack()
upper_green_s.pack()
upper_green_v.pack()

separator = ttk.Separator(master, orient='horizontal')
separator.pack(fill='x')

button = Button(master, text="Start process")
button.pack()

mainloop()
