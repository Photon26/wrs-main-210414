import modeling.geometricmodel as gm
import visualization.panda.world as wd
import cv2
import img_to_depth as itd
import time
from classdef import Lookuptable
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

frame = cv2.imread("cam2/7.jpg")
itd_cvter = itd.ImageToDepth()
depth, hm = itd_cvter.convert(frame)
s1, s2 = np.shape(hm)[:2]
#   plot the depth pic
fig = plt.figure()
# fig.set_size_inches(30,10)
ax = Axes3D(fig)
X = np.arange(0, s2)
Y = np.arange(0, s1)
X, Y = np.meshgrid(X, Y)
ax.set_zlim3d(0, 10)
ax.get_proj = lambda: np.dot(Axes3D.get_proj(ax), np.diag([1, 1, 0.3, 1]))
ax.plot_surface(X, Y, hm, rstride=1, cstride=1, cmap='rainbow')

plt.show()