#### Program for my research
#### generate many pose of graspping in both hands(for pulling rope) and pushing in left hand(for pushing object)
import visualization.panda.world as wd
import robotsim.grippers.robotiq85.robotiq85 as rtq85
import robotsim.robots.ur3_dual.ur3_dual as rbts
import modeling.collisionmodel as cm
import modeling.geometricmodel as gm
import basis.robot_math as rm
import math
import numpy as np
from pyquaternion import Quaternion
import copy

class PoseMaker(object):

    def __init__(self):
        self.rtq85 = rtq85.Robotiq85()
        self.rbt = rbts.UR3Dual()
        # # import manipulation.grip.robotiq85.robotiq85 as rtq85
        # # self.base = pandactrl.World(camp=[5000, 3000, 3000], lookatp=[0, 0, 700])
        # self.hndfa = rtq85.Robotiq85Factory()
        # self.rtq85 = self.hndfa.genHand()
        # self.rgthnd = self.hndfa.genHand()
        # self.lfthnd = self.hndfa.genHand()
        # self.rbt = robot.Ur3DualRobot(self.rgthnd, self.lfthnd)
        # self.rbtmg = robotmesh.Ur3DualMesh()
        # # self.obj = cm.CollisionModel(objinit="./objects/research_box.stl")

    def lftgrasppose(self):
        lftdirstart = 250
        lftverticalstart = lftdirstart + 90
        handrotrange = 5
        predefined_grasps_lft = []
        handdirect_lft = []
        predefined_grasps_lft.append(
            self.rtq85.grip_at(np.array([.005, .005, .005]), np.array([0, -1, 0]), np.array([1, 0, 0]),
                               jaw_width=self.rtq85.jaw_width_rng[1]))
        handdirect_lft.append([0, -1, 0])
        for i in range(8):
            predefined_grasps_lft.append(
                self.rtq85.grip_at(np.array([.005, .005, .005]),
                                   np.array([math.cos(math.radians(lftdirstart + i * handrotrange)),
                                             math.sin(math.radians(lftdirstart + i * handrotrange)), -.2]),
                                   np.array([math.cos(math.radians(lftverticalstart + i * handrotrange)),
                                             math.sin(math.radians(lftverticalstart + i * handrotrange)), 0]),
                                   jaw_width=self.rtq85.jaw_width_rng[0]))
            handdirect_lft.append([math.cos(math.radians(lftdirstart + i * handrotrange)),
                                   math.sin(math.radians(lftdirstart + i * handrotrange)), -.2])

        for i in range(8):
            predefined_grasps_lft.append(
                self.rtq85.grip_at(np.array([.005, .005, .005]),
                                   np.array([math.cos(math.radians(lftdirstart + i * handrotrange)),
                                             math.sin(math.radians(lftdirstart + i * handrotrange)), 0]),
                                   np.array([math.cos(math.radians(lftverticalstart + i * handrotrange)),
                                             math.sin(math.radians(lftverticalstart + i * handrotrange)), 0]),
                                   jaw_width=self.rtq85.jaw_width_rng[0]))
            handdirect_lft.append([math.cos(math.radians(lftdirstart + i * handrotrange)),
                                   math.sin(math.radians(lftdirstart + i * handrotrange)), 0])

        for i in range(8):
            predefined_grasps_lft.append(
                self.rtq85.grip_at(np.array([.005, .005, .005]),
                                   np.array([math.cos(math.radians(lftdirstart + i * handrotrange)),
                                             math.sin(math.radians(lftdirstart + i * handrotrange)), .2]),
                                   np.array([math.cos(math.radians(lftverticalstart + i * handrotrange)),
                                             math.sin(math.radians(lftverticalstart + i * handrotrange)), 0]),
                                   jaw_width=self.rtq85.jaw_width_rng[0]))
            handdirect_lft.append([math.cos(math.radians(lftdirstart + i * handrotrange)),
                                   math.sin(math.radians(lftdirstart + i * handrotrange)), .2])
        return predefined_grasps_lft, handdirect_lft

    def rgtgrasppose(self):
        rgtdirstart = 90
        rgtverticalstart = rgtdirstart - 90
        handrotrange = 5
        predefined_grasps_rgt = []
        handdirect_rgt = []
        predefined_grasps_rgt.append(
            self.rtq85.grip_at(np.array([.005, .005, .005]), np.array([0, 1, 0]), np.array([1, 0, 0]),
                               jaw_width=self.rtq85.jaw_width_rng[1]))
        handdirect_rgt.append([0, 1, 0])
        for i in range(4):
            predefined_grasps_rgt.append(
                self.rtq85.grip_at(np.array([.005, .005, .005]),
                                   np.array([math.cos(math.radians(rgtdirstart - i * handrotrange)),
                                             math.sin(math.radians(rgtdirstart - i * handrotrange)), -.1]),
                                   np.array([math.cos(math.radians(rgtverticalstart - i * handrotrange)),
                                             math.sin(math.radians(rgtverticalstart - i * handrotrange)), 0]),
                                   jaw_width=self.rtq85.jaw_width_rng[0]))
            handdirect_rgt.append([math.cos(math.radians(rgtdirstart - i * handrotrange)),
                                   math.sin(math.radians(rgtdirstart - i * handrotrange)), -.1])
        for i in range(4):
            predefined_grasps_rgt.append(
                self.rtq85.grip_at(np.array([.005, .005, .005]),
                                   np.array([math.cos(math.radians(rgtdirstart - i * handrotrange)),
                                             math.sin(math.radians(rgtdirstart - i * handrotrange)), 0]),
                                   np.array([math.cos(math.radians(rgtverticalstart - i * handrotrange)),
                                             math.sin(math.radians(rgtverticalstart - i * handrotrange)), 0]),
                                   jaw_width=self.rtq85.jaw_width_rng[0]))
            handdirect_rgt.append([math.cos(math.radians(rgtdirstart - i * handrotrange)),
                                   math.sin(math.radians(rgtdirstart - i * handrotrange)), 0])

            for i in range(4):
                predefined_grasps_rgt.append(
                    self.rtq85.grip_at(np.array([.005, .005, .005]),
                                       np.array([math.cos(math.radians(rgtdirstart - i * handrotrange)),
                                                 math.sin(math.radians(rgtdirstart - i * handrotrange)), .1]),
                                       np.array([math.cos(math.radians(rgtverticalstart - i * handrotrange)),
                                                 math.sin(math.radians(rgtverticalstart - i * handrotrange)), 0]),
                                       jaw_width=self.rtq85.jaw_width_rng[0]))
            handdirect_rgt.append([math.cos(math.radians(rgtdirstart - i * handrotrange)),
                                   math.sin(math.radians(rgtdirstart - i * handrotrange)), .1])
        return predefined_grasps_rgt, handdirect_rgt

    def pushpose(self, axisvec, pushpoint, toggle_debug=False):
        pushposelist = []
        pushpose_rotmatlist = []
        zaxis = np.array([0, 0, 1])
        axisvec_norm = np.linalg.norm(axisvec)  ## ??????????????????????????????
        theta = 5
        degree = 90  ## 30
        handrotate = 180  ## 30
        thetamax = 30  ## 60
        thetarange = int(thetamax / theta)
        degreerange = int(360 / degree)
        handrotaterange = int(360 / handrotate)
        for i in range(thetarange):
            referencevec = axisvec + (axisvec_norm * math.tan(math.radians(theta * (i + 1)))) * zaxis
            referencepoint = pushpoint + referencevec
            ## ????????????????????????????????????????????????????????????????????????????????????????????????
            q_refvec = Quaternion(0, referencepoint[0] - pushpoint[0], referencepoint[1] - pushpoint[1],
                                  referencepoint[2] - pushpoint[2])
            for j in range(degreerange):
                q_axis = Quaternion(axis=rm.unit_vector(axisvec), degrees=degree * (j + 1))  ## ???????????????????????????
                q_new = q_axis * q_refvec * q_axis.inverse
                ## ?????????????????????
                point = np.array([q_new[1] + pushpoint[0], q_new[2] + pushpoint[1], q_new[3] + pushpoint[2]])
                # base.pggen.plotSphere(base.render, pos=point, radius=10, rgba=[0,0,1,1])
                handdir = pushpoint - point
                handdir_projection = copy.copy(handdir)  ## xy?????????????????????
                handdir_projection[2] = 0
                handdir_projection = rm.unit_vector(handdir_projection)
                ## ???????????????????????????????????????????????????
                handdir = rm.unit_vector(handdir)  ## z
                thumb_verticalvec = np.cross(zaxis, handdir_projection)  ## x
                zaxis_hand = np.cross(handdir, thumb_verticalvec)  ## y
                # pushposelist.append(self.rtq85.approachAt(5,5,5,thumb_verticalvec[0], thumb_verticalvec[1], thumb_verticalvec[2],
                #                                       handdir[0], handdir[1], handdir[2], jaw_width=0))
                ## ???????????????????????????-90????????????????????????????????????
                for k in range(handrotaterange):
                    handrotmat = np.empty((0, 3))
                    ## test
                    # handrotmat = np.append(handrotmat, np.array([handdir]), axis=0)
                    # handrotmat = np.append(handrotmat, np.array([thumb_verticalvec]), axis=0)
                    # handrotmat = np.append(handrotmat, np.array([zaxis_hand]), axis=0)
                    handrotmat = np.append(handrotmat, np.array([thumb_verticalvec]), axis=0)
                    handrotmat = np.append(handrotmat, np.array([zaxis_hand]), axis=0)
                    handrotmat = np.append(handrotmat, np.array([handdir]), axis=0)
                    handrotmat = handrotmat.T
                    handrotmat = np.dot(rm.rotmat_from_axangle(handrotmat[:, 2], - math.radians(handrotate * k)), handrotmat)
                    pushposelist.append(
                        self.rtq85.grip_at(np.array([.005, .005, .005]),
                                           np.array([handrotmat[:, 2][0], handrotmat[:, 2][1], handrotmat[:, 2][2]]),
                                           np.array([handrotmat[:, 0][0], handrotmat[:, 0][1], handrotmat[:, 0][2]]),
                                           jaw_width=self.rtq85.jaw_width_rng[0]))
                    if toggle_debug:
                        self.rtq85.copy().gen_meshmodel().attach_to(base)
                    pushpose_rotmatlist.append(handrotmat)
        return pushpose_rotmatlist


if __name__ == "__main__":
    base = wd.World(cam_pos=[.3, 0, .3], lookat_pos=[0, 0, 0])
    gm.gen_frame().attach_to(base)
    ## ????????????????????????????????????
    axisvec = np.array([0, 1, 0])
    pushpoint = np.array([0, 0, 0])
    p_maker = PoseMaker()
    rotmatlist = p_maker.pushpose(axisvec, pushpoint, toggle_debug=True)
    print("len", len(rotmatlist))
    base.run()
