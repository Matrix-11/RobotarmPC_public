#!/usr/bin/env python

from math import pi
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH, RevoluteMDH, PrismaticMDH


class MYROBOT(DHRobot):

    def __init__(self):
        deg = pi / 180

        L0 = RevoluteDH(
            d=0.1, alpha=-pi / 2, # d= 0.1
            qlim=[0 * deg, 360 * deg])

        L1 = RevoluteDH(
            d=0, a=0.194, alpha=0, offset=-pi / 2,
            qlim=[-90 * deg, 90 * deg])

        L2 = RevoluteDH(
            d=0, a=0.211740407, offset=1.317799508,
            qlim=[-90 * deg, 90 * deg])

        L3 = RevoluteDH(
            d=0, a=0.0, offset=-1.317799508, alpha=-pi / 2,
            qlim=[0 * deg, 90 * deg])

        L4 = RevoluteDH(
            a=0, d=0.099175, offset=0, alpha=0,  # d=0.035175 ohne bracket # 0.058675 mit bracket
            qlim=[-180 * deg, 180 * deg])

        # L4 = E

        super().__init__(
            [L0, L1, L2, L3, L4],
            name="MYROBOT",
            manufacturer="ME")

        self.qz = np.zeros(5)

        self.addconfiguration("qz", self.qz)
        self.addconfiguration_attr("qs", np.array([0, -pi / 2, pi / 2, -pi / 2, 0]))  # Parked Position

        # zero angles, L shaped pose
        # self._MYCONFIG = np.array([1, 2, 3, 4, 5, 6])  # create instance attribute


@property
def MYCONFIG(self):
    return self._MYCONFIG


if __name__ == '__main__':
    robot = MYROBOT()
    print(robot)
