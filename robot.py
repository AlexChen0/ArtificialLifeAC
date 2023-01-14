import pybullet as p
import pyrosim.pyrosim as pyrosim
from sensor import SENSOR
from motor import MOTOR
import constants as c

class ROBOT:

    def __init__(self):
        self.sensors = {}
        self.motors = {}
        self.robotId = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()

    def Prepare_To_Sense(self):
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def Sense(self, t):
        for s in self.sensors:
            s.Get_Value(t)

    def Prepare_To_Act(self):
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName, c.ampBack, c.freqBack, c.phaseOffsetBack)

    def Act(self, t):
        for m in self.motors:
            m.SetValue(t, self.robotId)

    def getSensors(self):
        return self.sensors




