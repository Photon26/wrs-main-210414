import cv2
import numpy as np

def fishye_calib(img, para):
    K, D, DIM = para
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return undistorted_img

import pickle

para = pickle.load(open("cam2/calib.pkl", "rb"))
img = cv2.imread("cam2/4.jpg")
img2 = fishye_calib(img, para)
cv2.imshow("tst", img2)
cv2.waitKey(0)