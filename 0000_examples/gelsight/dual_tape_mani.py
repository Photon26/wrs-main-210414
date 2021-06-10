import pickle
import robotsim.robots.ur3_dual.ur3_dual as ur3d
import robotcon.ur.ur3_dual_x as ur3dx
import visualization.panda.world as wd
import modeling.geometricmodel as gm
import numpy as np
import cv2
import img_to_depth as itd

inner_rad = 75/2
outer_rad = 99/2

robot_instance = ur3d.UR3Dual()
ur_dual_x = ur3dx.UR3DualX(lft_robot_ip='10.2.0.50', rgt_robot_ip='10.2.0.51', pc_ip='10.2.0.100')

#
# jnts = ur_dual_x.lft_arm_hnd.get_jnt_values()
# robot_instance.lft_arm.fk(jnts)
# pose = robot_instance.lft_arm.get_gl_tcp()
# print(pose)

base = wd.World(cam_pos=[2, 1, 3], lookat_pos=[0, 0, 1.1])
gm.gen_frame().attach_to(base)
component_name = 'lft_arm'
robot_s = ur3d.UR3Dual()

pose_hnd = robot_s.get_gl_tcp(manipulator_name="lft_arm")
print(pose_hnd)

ini_pos = np.array([ 0.3, 0,  1.5])
ini_rot = np.array([[ 1, 0,  0],
       [ 0 , 0, -1],
       [ 0,  1,  0]])
newjnt = robot_s.ik("lft_arm", ini_pos, ini_rot)
robot_s.fk(component_name, newjnt)
robot_meshmodel = robot_s.gen_meshmodel(toggle_tcpcs=True)
robot_meshmodel.attach_to(base)
center = ini_pos + np.dot(ini_rot, np.array([0, -0.001*outer_rad, 0]))

for theta in range(1, 4):
    rotmat = np.array([[np.cos(np.pi/4*theta),0, np.sin(np.pi/4*theta)], [0,1,0],[-np.sin(theta),0,np.cos(np.pi/4*theta)]])
    rot = np.dot(rotmat, ini_rot)
    pos = center + np.dot(rot,np.array([0, 0.001*outer_rad, 0]))
    gm.gen_frame(pos, rot).attach_to(base)

    newjnt = robot_s.ik("lft_arm", pos, rot)
    robot_s.fk(component_name, newjnt)
    robot_meshmodel = robot_s.gen_meshmodel(toggle_tcpcs=False)
    robot_meshmodel.attach_to(base)

base.run()
