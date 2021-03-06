import math
import numpy as np
import visualization.panda.world as wd
import modeling.geometricmodel as gm
import modeling.collisionmodel as cm
import robotsim.robots.ur3_dual.ur3_dual as ur3d
import motion.probabilistic.rrt_connect as rrtc

base = wd.World(cam_pos=[2, 1, 3], lookat_pos=[0, 0, 1.1])
gm.gen_frame().attach_to(base)
# object
# object = cm.CollisionModel("./objects/bunnysim.stl")
# object.set_pos(np.array([.55, -.3, 1.3]))
# object.set_rgba([.5, .7, .3, 1])
# object.attach_to(base)
# robot_s
robot_s = ur3d.UR3Dual()

# possible right goal np.array([0, -math.pi/4, 0, math.p i/2, math.pi/2, math.pi / 6])
# possible left goal np.array([0, -math.pi / 2, -math.pi/3, -math.pi / 2, math.pi / 6, math.pi / 6])

# rrtc_planner = rrtc.RRTConnect(robot_s)
# path = rrtc_planner.plan(component_name=component_name,
#                          start_conf=robot_s.lft_arm.homeconf,
#                          goal_conf=np.array([0, -math.pi / 2, -math.pi/3, -math.pi / 2, math.pi / 6, math.pi / 6]),
#                          # obstacle_list=[object],
#                          ext_dist=.2,
#                          rand_rate=70,
#                          maxtime=300)
# print(path)
# for pose in path:
#     print(pose)
#     robot_s.fk(component_name, pose)
#     robot_meshmodel = robot_s.gen_meshmodel(toggle_tcpcs=False)
#     robot_meshmodel.attach_to(base)
#     robot_s.gen_stickmodel().attach_to(base)
pose_hnd = robot_s.get_gl_tcp(manipulator_name="lft_arm")
print(pose_hnd)

ini_pos_lft = np.array([ 0.3, 0,  1.4])
ini_rot_lft = np.array([[ 1, 0,  0],
       [ 0 , 0, -1],
       [ 0,  1,  0]])
newjnt = robot_s.ik("lft_arm", ini_pos_lft, ini_rot_lft)
robot_s.fk(component_name, newjnt)
robot_meshmodel = robot_s.gen_meshmodel(toggle_tcpcs=True)
robot_meshmodel.attach_to(base)


for theta in range(0, 4):

    rotmat = np.array([[np.cos(np.pi/4*theta),0, np.sin(np.pi/4*theta)], [0,1,0],[-np.sin(theta),0,np.cos(np.pi/4*theta)]])
    rot = np.dot(rotmat, ini_rot)
    newjnt = robot_s.ik("lft_arm", ini_pos, rot)
    robot_s.fk(component_name, newjnt)
    robot_meshmodel = robot_s.gen_meshmodel(toggle_tcpcs=True)
    robot_meshmodel.attach_to(base)

base.run()
