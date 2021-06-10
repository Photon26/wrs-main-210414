import pickle
import robotsim.robots.ur3_dual.ur3_dual as ur3d
import robotcon.ur.ur3_dual_x as ur3dx
import numpy as np
import cv2
import img_to_depth as itd

inner_rad = 75
outer_rad = 99

robot_instance = ur3d.UR3Dual()
ur_dual_x = ur3dx.UR3DualX(lft_robot_ip='10.2.0.50', rgt_robot_ip='10.2.0.51', pc_ip='10.2.0.100')
# ini_pos_rgt = pickle.load(open("ini_pos_rgt.pkl","rb"))
# ur_dual_x.rgt_arm_hnd.move_jnts(ini_pos_rgt)
#
# robot_instance.fk(component_name="rgt_arm", jnt_values=ini_pos_rgt)
# pose_hnd = robot_instance.get_gl_tcp(manipulator_name="rgt_arm")

jnts = ur_dual_x.lft_arm_hnd.get_jnt_values()
robot_instance.lft_arm.fk(jnts)
pose = robot_instance.lft_arm.get_gl_tcp()
print(pose)