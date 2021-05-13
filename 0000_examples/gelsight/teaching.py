import math
import numpy as np
import visualization.panda.world as wd
import modeling.geometricmodel as gm
import modeling.collisionmodel as cm
import robotsim.robots.ur3_dual.ur3_dual as ur3d
import motion.probabilistic.rrt_connect as rrtc
import robotcon.ur.ur3_dual_x as ur3dx

base = wd.World(cam_pos=[2, 1, 3], lookat_pos=[0, 0, 1.1])
gm.gen_frame().attach_to(base)
# robot_s
component_name = 'lft_arm'
robot_instance = ur3d.UR3Dual()

# init_lft_arm_jnt_values = robot_s.lft_arm.get_jnt_values()
# init_rgt_arm_jnt_values = robot_s.rgt_arm.get_jnt_values()
# full_jnt_values = np.hstack((init_lft_arm_jnt_values, init_rgt_arm_jnt_values))
full_jnt_values = np.hstack((robot_instance.lft_arm.homeconf, robot_instance.rgt_arm.homeconf))
goal_lft_arm_jnt_values = np.array([0, -math.pi / 2, -math.pi/3, -math.pi / 2, math.pi / 6, math.pi / 6])
goal_rgt_arm_jnt_values = np.array([0, -math.pi/4, 0, math.pi/2, math.pi/2, math.pi / 6])
robot_instance.fk(component_name="lft_arm", jnt_values = np.array([0, -math.pi / 2, -math.pi/3, -math.pi / 2, math.pi / 6, math.pi / 6]))
pos = robot_instance.lft_arm.get_gl_tcp()
print(pos)
robot_meshmodel = robot_instance.gen_meshmodel()
robot_meshmodel.attach_to(base)
gm.gen_sphere(pos[0]).attach_to(base)

# rrtc_planner = rrtc.RRTConnect(robot_instance)
# path = rrtc_planner.plan(component_name=component_name,
#                          start_conf=full_jnt_values,
#                          goal_conf=np.hstack((goal_lft_arm_jnt_values, goal_rgt_arm_jnt_values)),
#                          obstacle_list=[object],
#                          ext_dist=.2,
#                          rand_rate=70,
#                          maxtime=300)
#
# print(path)
# for pose in path:
#     print(pose)
#     robot_instance.fk(component_name, pose)
#     robot_meshmodel = robot_instance.gen_meshmodel()
#     robot_meshmodel.attach_to(base)
#     robot_instance.gen_stickmodel().attach_to(base)

# ur_dual_x = ur3dx.UR3DualX(lft_robot_ip='10.2.0.50', rgt_robot_ip='10.2.0.51', pc_ip='10.2.0.100')
# ur_dual_x.move_jntspace_path(path)

base.run()