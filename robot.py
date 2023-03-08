import pybullet as p
import pyrosim.pyrosim as pyrosim
from sensor import SENSOR
from motor import MOTOR
import constants as c
from pyrosim.neuralNetwork import NEURAL_NETWORK
import os

class ROBOT:

    def __init__(self, targetID):
        print("robot made")
        self.targetID = targetID
        self.sensors = {}
        self.motors = {}
        self.robotId = p.loadURDF("body" + str(targetID) + ".urdf", flags=p.URDF_USE_SELF_COLLISION + p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)
        try:
            os.system("rm body" + str(targetID) + ".urdf")
        except:
            print("no body urdf?")
            pass
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()
        self.nn = NEURAL_NETWORK("brain" + str(targetID) + ".nndf")
        try:
            os.system("rm brain" + str(targetID) + ".nndf")
        except:
            print("no brain nndf?")
            pass

    def Prepare_To_Sense(self):
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def Sense(self, t):
        for s in self.sensors:
            self.sensors[s].Get_Value(t)

    def Prepare_To_Act(self):
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName, c.ampBack, c.freqBack, c.phaseOffsetBack)

    def Act(self, t):
        # print(self.motors.keys())
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(neuronName) * c.motorJointRange
                self.motors[jointName].SetValue(self.robotId, desiredAngle)

    def getSensors(self):
        return self.sensors

    def Think(self):
        self.nn.Update()

    def Get_Fitness(self):
        stateOfLinkZero = p.getLinkState(self.robotId, 0)
        xposLinkZero = stateOfLinkZero[0][0] * -1
        f = open("tmp" + str(self.targetID) + ".txt", "w")
        f.write(str(xposLinkZero))
        f.close()
        os.system("mv " + "tmp" + str(self.targetID) + ".txt" + " fitness" + str(self.targetID) + ".txt")



