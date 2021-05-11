import modeling.geometricmodel as gm
import visualization.panda.world as wd
import cv2
import img_to_depth as itd
import time
from classdef import Lookuptable
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from panda3d.core import NodePath
import visualization.panda.world as world
from modeling.geometricmodel import gen_pointcloud

cam = cv2.VideoCapture(0)
while (True):
    return_value, image = cam.read()

    # cv2.imshow("tst", image)
    # key = cv2.waitKey(0)
    # image = cv2.imread("cam2/tst.jpg")

    frame = image
    itd_cvter = itd.ImageToDepth()
    depth, hm = itd_cvter.convert(frame)

    #   mathplot
    # s1, s2 = np.shape(hm)[:2]
    # #   plot the depth pic
    # fig = plt.figure()
    # # fig.set_size_inches(30,10)
    # ax = Axes3D(fig)
    # X = np.arange(0, s2)
    # Y = np.arange(0, s1)
    # X, Y = np.meshgrid(X, Y)
    # ax.set_zlim3d(0, 10)
    # ax.get_proj = lambda: np.dot(Axes3D.get_proj(ax), np.diag([1, 1, 0.3, 1]))
    # ax.plot_surface(X, Y, hm, rstride=1, cstride=1, cmap='rainbow')
    #
    # plt.show()

    #   panda3d show
    # base = world.World(cam_pos=[.03, .03, .07], lookat_pos=[0.015, 0.015, 0])
    # pointcloud = None
    #
    # pcdm = []
    # pcdm.append(None)
    # pcdm[0] = gm.GeometricModel(depth*.001)
    # pcdm[0].attach_to(base)
    #
    # base.run()

    depth_max = np.max(hm)
    hm_map = hm / depth_max * 255
    # print(np.shape(hm_map))
    hm_map = hm_map.astype('uint8')
    # cv2.imshow("tst", hm_map)
    # cv2.waitKey(0)
    # contours, hierarchy = cv2.findContours(hm_map, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # hm_cvt = cv2.cvtColor(hm_map, cv2.COLOR_GRAY2RGB)
    # cv2.drawContours(hm_cvt, contours, -1, (0, 0, 255), 3)
    # print(contours)
    img = hm_map

    # img = cv2.imread("cam2/")
    # img = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)

    #
    # img = cv2.imread('messi5.jpg',0)
    f = np.fft.fft2(img)
    fshift = np.fft.fftshift(f)
    magnitude_spectrum = 100 * np.log(np.abs(fshift))

    # plt.subplot(121), plt.imshow(img, cmap='gray')
    # plt.title('Input Image'), plt.xticks([]), plt.yticks([])
    # plt.subplot(122), plt.imshow(magnitudFe_spectrum, cmap='gray')
    # plt.title('Magnitude Spectrum'), plt.xticks([]), plt.yticks([])
    # plt.show()

    rows, cols = img.shape
    crow, ccol = int(rows / 2), int(cols / 2)
    p = 30
    fshift[crow - p:crow + p, ccol - p:ccol + p] = 0
    f_ishift = np.fft.ifftshift(fshift)
    img_back = np.fft.ifft2(f_ishift)
    img_back = np.abs(img_back)

    # plt.subplot(131), plt.imshow(img, cmap='gray')
    # plt.title('Input Image'), plt.xticks([]), plt.yticks([])
    # plt.subplot(132), plt.imshow(img_back, cmap='gray')
    # plt.title('Image after HPF'), plt.xticks([]), plt.yticks([])
    # plt.subplot(133), plt.imshow(img_back)
    # plt.title('Result in JET'), plt.xticks([]), plt.yticks([])
    #
    # plt.show()

    # read image
    # img = cv2.imread('fft.png')

    # # convert to grayscale
    img_back = (img_back / np.amax(img_back) * 255).astype("uint8")

    # cv2.imshow("tst", img_back)

    # # img_back = cv2.cvtColor(img_back,cv2.COLOR_BGR2GRAY)
    # gray = 255 - img_back
    #
    # # threshold
    # thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 13, 4)
    # thresh = 255 - thresh
    #
    # # apply close to connect the white areas
    # kernel = np.ones((3,3), np.uint8)
    # morph = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
    # kernel = np.ones((1,9), np.uint8)
    # morph = cv2.morphologyEx(morph, cv2.MORPH_CLOSE, kernel)

    # apply canny edge detection
    edges = cv2.Canny(img_back, 150, 200)

    # get hough lines
    result = img.copy()
    lines = cv2.HoughLines(edges, 1, np.pi / 180, 50)
    # Draw line on the image
    result = cv2.cvtColor(result, cv2.COLOR_GRAY2RGB)
    for rho, theta in lines[0]:
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        cv2.line(result, (x1, y1), (x2, y2), (0, 0, 255), 1)

    # # save resulting images
    # # cv2.imwrite('fft_thresh.jpg',thresh)
    # # cv2.imwrite('fft_morph.jpg',morph)
    # cv2.imwrite('fft_edges.jpg', edges)
    # cv2.imwrite('fft_line.jpg', result)
    #
    # # show thresh and result
    # # cv2.imshow("thresh", thresh)
    # # cv2.imshow("morph", morph)
    # cv2.imshow("edges", edges)
    cv2.imshow("result", result)
    key = cv2.waitKey(10)
    if int(key) == 113:
        break
cam.release()
