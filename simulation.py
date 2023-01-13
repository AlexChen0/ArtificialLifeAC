from world import WORLD
from robot import ROBOT
import pybullet as p
import pyrosim.pyrosim as pyrosim
import pybullet_data
import constants as c
import time
import numpy as np

class SIMULATION:

    def __init__(self):
        self.physicsClient = p.connect(p.GUI)
        self.world = WORLD()
        self.robot = ROBOT()
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, c.gravity)

    def run(self):
        for i in range(c.numIterations):
            p.stepSimulation()
            self.robot.Sense(i)
            self.robot.Act(i)
            time.sleep(c.frameRate)

    def __del__(self):
        for i in self.robot.sensors:
            np.save(i.linkName + "Val.npy", i.values)
        p.disconnect()
