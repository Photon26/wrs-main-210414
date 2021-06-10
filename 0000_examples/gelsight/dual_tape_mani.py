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



base = wd.World(cam_pos=[2, 1, 3], lookat_pos=[0, 0, 1.1])
gm.gen_frame().attach_to(base)
robot_s = ur3d.UR3Dual()

pose_hnd = robot_s.get_gl_tcp(manipulator_name="lft_arm")
print(pose_hnd)

ini_pos = np.array([ 0.3, 0,  1.5])
ini_rot_lft = np.array([[ 1, 0,  0],
       [ 0 , 0, -1],
       [ 0,  1,  0]])
newjnt = robot_s.ik("lft_arm", ini_pos, ini_rot_lft)
robot_s.fk("lft_arm", newjnt)
robot_meshmodel = robot_s.gen_meshmodel(toggle_tcpcs=True)
robot_meshmodel.attach_to(base)
center = ini_pos + np.dot(ini_rot_lft, np.array([0, -0.001*outer_rad, 0]))

ini_rot_rgt = np.array([[ -1, 0,  0],
       [ 0 , 0, 1],
       [ 0,  1,  0]])

jnt_list = []
for theta in range(0,13):
    rotmat = np.array([[np.cos(np.pi / 24 * theta + np.pi/4), 0, np.sin(np.pi / 24 * theta + np.pi/4)], [0, 1, 0],
                       [-np.sin(theta), 0, np.cos(np.pi / 4 * theta)]])
    rot = np.dot(rotmat, ini_rot_rgt)
    pos = center + np.dot(rot, np.array([0, 0.001 * outer_rad, 0]))
    newjnt_rgt = robot_s.ik("rgt_arm", pos, rot)
    jnt_list.append(newjnt_rgt)

#  rgt hand hold the tape
ini_jnt_rgt = jnt_list[0]

#  loose lft hand a bit

#  rotate rgt hand
for jnt in jnt_list:
    ur_dual_x.rgt_arm_hnd.move_jnts(jnt)


# for theta in range(1, 4):
#     rotmat = np.array([[np.cos(np.pi/4*theta),0, np.sin(np.pi/4*theta)], [0,1,0],[-np.sin(theta),0,np.cos(np.pi/4*theta)]])
#     rot = np.dot(rotmat, ini_rot)
#     pos = center + np.dot(rot,np.array([0, 0.001*outer_rad, 0]))
#     gm.gen_frame(pos, rot).attach_to(base)
#     newjnt = robot_s.ik("lft_arm", pos, rot)
#     robot_s.fk(component_name, newjnt)
#     robot_meshmodel = robot_s.gen_meshmodel(toggle_tcpcs=False)
#     robot_meshmodel.attach_to(base)

base.run()
