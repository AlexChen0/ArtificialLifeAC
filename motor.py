import numpy as np
import constants as c
import pyrosim.pyrosim as pyrosim
import pybullet as p


class MOTOR:

    def __init__(self, jointName, amp, freq, offset):
        self.jointName = jointName
        self.motorValues = np.zeros(c.numIterations)
        self.amplitude = amp
        self.frequency = freq
        if self.jointName == "Torso_BackLeg":
            self.frequency = self.frequency / 2
        self.offset = offset

    def SetValue(self, robotID, t):
        pyrosim.Set_Motor_For_Joint(bodyIndex=robotID, jointName=self.jointName,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=t, maxForce=c.maxForce)

