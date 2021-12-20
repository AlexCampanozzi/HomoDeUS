import os
import uuid
import cv2
import yaml
import time
import numpy as np
from scipy import ndimage
from progress.bar import Bar


def cut_video(filename, cut = 10):
    path = "./process/videos/" + filename

    cut_counter = 0
    cap = cv2.VideoCapture(path)

    while(cap.isOpened()):

        ret, frame = cap.read()

        if ret == True:

            if cut_counter % cut == 0:
                folder = (filename.split("."))[0]
                dir_path = "./process/raw_objects/" + folder + "/"

                if not os.path.isdir(dir_path):
                    os.mkdir(dir_path)

                frame_path = dir_path + str(cut_counter) + ".jpg"
                cv2.imwrite(frame_path, frame)

            cut_counter += 1

        else:
            break

    cap.release()


#-------------------------------------------------------------------------------
def main():

    video_files = os.listdir("./process/videos/")

    for file in video_files:
        cut_video(file)




if __name__ == "__main__":
    main()
