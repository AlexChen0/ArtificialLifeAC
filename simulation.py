from world import WORLD
from robot import ROBOT
import pybullet as p
import pyrosim.pyrosim as pyrosim
import pybullet_data
import constants as c
import time
import numpy as np


class SIMULATION:

    def __init__(self, DoG, targetID):
        self.DoG = DoG
        self.targetID = targetID
        if DoG == "DIRECT":
            self.physicsClient = p.connect(p.DIRECT)
        else:
            self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, c.gravity * 10)
        self.world = WORLD()
        self.robot = ROBOT(self.targetID)

    def run(self):
        for i in range(c.numIterations):
            p.stepSimulation()
            self.robot.Sense(i)
            self.robot.Think()
            self.robot.Act(i)
            if self.DoG == "GUI":
                time.sleep(c.frameRate)
            else:
                pass

    def Get_Fitness(self):
        self.robot.Get_Fitness()

    def __del__(self):
        p.disconnect()
