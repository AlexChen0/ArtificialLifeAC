import numpy as np
import constants as c
import pyrosim.pyrosim as pyrosim
import pybullet as p


class MOTOR:

    def __init__(self, jointName, amp, freq, offset):
        self.jointName = jointName
        self.motorValues = np.zeros(c.numIterations)
        self.Prepare_To_Act()
        self.amplitude = amp
        self.frequency = freq
        self.offset = offset

    def Prepare_To_Act(self):
        self.motorValues = np.sin((np.linspace(0, np.pi * 2, c.numIterations) * self.frequency + self.offset)) * self.amplitude

    def SetValue(self, t, robotID):
        pyrosim.Set_Motor_For_Joint(bodyIndex=robotID, jointName=self.jointName,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=self.motorValues[t], maxForce=c.maxForce)

    def Save_Values(self):
        np.save(self.jointName + "Val.npy", self.motorValues)
