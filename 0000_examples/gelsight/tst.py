import cv2
import numpy as np
from calibrate_gelsight import takeimg, fishye_calib, imgborder
import pickle

# para = pickle.load(open("cam2/cam3_calib.pkl", "rb"))
# img = cv2.imread("fisheye/1tst.jpg")
# border = imgborder(img, 1, campara=para)
# print(border)
takeimg("cam2",0, 0, "tst3")
