import math
import numpy as np
import pyrosim.pyrosim as pyrosim
import os
import random
import time
import constants as c


class SOLUTION:
    def __init__(self, ID):
        self.weights = np.random.rand(c.numSensorNeurons, c.numMotorNeurons)
        self.numSensors = -1
        self.numMotors = -1
        self.numSegments = -1
        self.sensorBlocks = []
        self.weights = self.weights * 2 - 1
        self.fitness = math.inf
        self.ID = ID
        # print(self.weights)

    def Set_ID(self, ID):
        self.ID = ID

    def Start_Simulation(self, DoG):
        self.Create_World()
        self.Create_Body()
        self.Create_Brain(self.numSegments)
        statement = "python3 simulate.py " + DoG + " " + str(self.ID) + " 2&>1 &"
        os.system(statement)

    def Wait_For_Simulation_To_End(self):
        while not os.path.exists("fitness" + str(self.ID) + ".txt"):
            time.sleep(0.01)
        f = open("fitness" + str(self.ID) + ".txt", "r")
        self.fitness = float(f.read())
        f.close()
        os.system("rm fitness" + str(self.ID) + ".txt")

    def Mutate(self):
        randRow = random.randint(0, self.numSensors - 1)
        randCol = random.randint(0, self.numMotors - 1)
        # print(self.weights)
        self.weights[randRow][randCol] = random.random() * 2 - 1
        # print(self.weights)

    def Create_World(self):
        length = 1
        width = 1
        height = 1
        x = -2
        y = 2
        z = 0.5
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name="Box", pos=[x, y, z], size=[length, width, height], s1='<material name="Cyan">', s2='    <color rgba="0 1.0 1.0 1.0"/>')
        pyrosim.End()

    def Create_Body(self):
        # length = 1
        # width = 1
        # height = 1
        # x = 0
        # y = 0
        # z = 1
        # legsize = [.15, .15, .3]
        self.numSegments = random.randint(8, 20)
        # synthesize sensors and motors here
        # sensorProbability = random.random()
        # let's set a minimum probability
        # while sensorProbability < .5:
        #     sensorProbability = random.random()
        sensorProbability = .45
        for i in range(self.numSegments):
            if random.random() < sensorProbability:
                self.sensorBlocks.append(i)
        self.numSensors = len(self.sensorBlocks)
        print("numSegments: ", self.numSegments)
        pyrosim.Start_URDF("body.urdf")
        # dummy nexts used to figure out positional garbage
        nextX = 0
        nextY = 0
        nextZ = 0
        for i in range(self.numSegments):
            sizeX = nextX
            sizeY = nextY
            sizeZ = nextZ
            JointName = str(i) + "_" + str(i + 1)
            if i == 0:
                # first.
                # create initial sizes and get next sizes
                sizeX = random.random() * 2
                sizeY = random.random() * 2
                sizeZ = random.random() * 2
                nextX = random.random() * 2
                nextY = random.random() * 2
                nextZ = random.random() * 2
                if i in self.sensorBlocks:
                    pyrosim.Send_Cube(name=str(i), pos=[0, 0, sizeZ / 2], size=[sizeX, sizeY, sizeZ],
                                      s1='<material name="Green">', s2='    <color rgba="0 1.0 0.0 1.0"/>')
                else:
                    pyrosim.Send_Cube(name=str(i), pos=[0, 0, sizeZ / 2], size=[sizeX, sizeY, sizeZ],
                                      s1='<material name="Cyan">', s2='    <color rgba="0 1.0 1.0 1.0"/>')
                pyrosim.Send_Joint(name=JointName, parent=str(i), child=str(i + 1), type="revolute",
                                   position=[sizeX/2 - nextX, 0, 0], jointAxis="0 0 1")
                # joint gotten, now done
            elif i == self.numSegments - 1:
                # last
                # position still 0 0 0 due to relative nature, but no more need to apply joint
                if i in self.sensorBlocks:
                    pyrosim.Send_Cube(name=str(i), pos=[0, 0, sizeZ / 2], size=[sizeX, sizeY, sizeZ],
                                      s1='<material name="Green">', s2='    <color rgba="0 1.0 0.0 1.0"/>')
                else:
                    pyrosim.Send_Cube(name=str(i), pos=[0, 0, sizeZ / 2], size=[sizeX, sizeY, sizeZ],
                                      s1='<material name="Cyan">', s2='    <color rgba="0 1.0 1.0 1.0"/>')
            else:
                # in the middle
                # position should be 0 0 0 thanks to joint stuff already figured out
                if i in self.sensorBlocks:
                    pyrosim.Send_Cube(name=str(i), pos=[0, 0, sizeZ / 2], size=[sizeX, sizeY, sizeZ],
                                      s1='<material name="Green">', s2='    <color rgba="0 1.0 0.0 1.0"/>')
                else:
                    pyrosim.Send_Cube(name=str(i), pos=[0, 0, sizeZ / 2], size=[sizeX, sizeY, sizeZ],
                                      s1='<material name="Cyan">', s2='    <color rgba="0 1.0 1.0 1.0"/>')
                nextX = random.random() * 2
                nextY = random.random() * 2
                nextZ = random.random() * 2
                pyrosim.Send_Joint(name=JointName, parent=str(i), child=str(i + 1), type="revolute",
                                   position=[sizeX / 2 - nextX, 0, 0], jointAxis="0 0 1")
        pyrosim.End()

    def Create_Brain(self, segmentCount):
        pyrosim.Start_NeuralNetwork("brain" + str(self.ID) + ".nndf")
        # let's set a minimum probability
        motorProbability = random.random()
        while motorProbability < .5:
            motorProbability = random.random()
        sensorCount = 0
        motorCount = 0
        for i in range(segmentCount):
            # only send a sensor or a motor if we roll lower than the set probability
            if i in self.sensorBlocks:
                # add the sensor
                pyrosim.Send_Sensor_Neuron(name=sensorCount, linkName=str(i))
                sensorCount += 1
        # same for motor
        for i in range(segmentCount):
            # only send a sensor or a motor if we roll lower than the set probability
            if random.random() < motorProbability and i < segmentCount - 1:
                # add the sensor
                JointName = str(i) + "_" + str(i + 1)
                pyrosim.Send_Motor_Neuron(name=sensorCount + motorCount, jointName=JointName)
                motorCount += 1
        # set self.weights here now
        self.numMotors = motorCount
        self.weights = np.random.rand(sensorCount, motorCount)
        self.weights = self.weights * 2 - 1
        for i in range(sensorCount):  # current row
            for j in range(motorCount):  # current column
                pyrosim.Send_Synapse(sourceNeuronName=i, targetNeuronName=j + sensorCount,
                                     weight=self.weights[i][j])
        print("sensor count:", self.numSensors)
        print("motor count:", self.numMotors)
        pyrosim.End()
