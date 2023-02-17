import math
import numpy as np
import pyrosim.pyrosim as pyrosim
import os
import random
import time
import constants as c
from BodyCube import BodyCube


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
        self.blocks = []
        self.joints = []
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
        self.numSegments = random.randint(5, 10)
        sensorProbability = .65
        self.sensorBlocks = []
        self.joints = []
        self.blocks = []
        for i in range(self.numSegments):
            if random.random() < sensorProbability:
                self.sensorBlocks.append(i)
        self.numSensors = len(self.sensorBlocks)
        print("numSegments: ", self.numSegments)
        pyrosim.Start_URDF("body.urdf")
        # dummy nexts used to figure out positional garbage
        nextOccupied = -1
        TargetParentCube = -1
        for i in range(self.numSegments):
            sizeX = random.random() * .8 + .2
            sizeY = random.random() * .8 + .2
            sizeZ = random.random() * .8 + .2
            if i == 0:
                # first.
                # create initial sizes and get next sizes
                if i in self.sensorBlocks:
                    self.blocks.append(BodyCube(name=str(i), position=[0, 0, sizeZ / 2], size=[sizeX, sizeY, sizeZ],
                                                s1='<material name="Green">', s2='    <color rgba="0 1.0 0.0 1.0"/>'))
                else:
                    self.blocks.append(BodyCube(name=str(i), position=[0, 0, sizeZ / 2], size=[sizeX, sizeY, sizeZ],
                                                s1='<material name="Cyan">', s2='    <color rgba="0 1.0 1.0 1.0"/>'))
            else:
                # in the middle and for the last
                # position should be 0 0 0 thanks to joint stuff already figured out
                # that said we need to figure out from the next target block where to assemble this one
                # joints do not need to be tracked by BodyCube, since they are guaranteed between 2 cubes
                # that said, now we need to decide which is the next segment that will be occupied
                # therefore, we no longer calculate where the joint is here, and we do it before calling cubes
                # but watch out: we need to figure out where the next cube goes too -- who's the parent?
                targetParentCube = random.randint(0, len(self.blocks) - 1)
                while self.blocks[targetParentCube].occupied == [1, 1, 1, 1, 1, 1]:
                    # guaranteed to not inf loop, mathematically impossible to have all cubes all occupied
                    targetParentCube = random.randint(0, len(self.blocks))
                openIndicies = [i for i in range(len(self.blocks[targetParentCube].occupied))
                                if self.blocks[targetParentCube].occupied[i] == 0]

                nextOccupied = random.choice(openIndicies)
                pos, jA = self.createJointArgument([sizeX, sizeY, sizeZ], self.blocks[targetParentCube].size, nextOccupied)
                # where the joint is is decided by nextOccupied, since we are either going to be making it in one of the
                # 6 faces of the block, now we set that to be occupied
                self.blocks[targetParentCube].setOccupied(nextOccupied)
                JointName = str(targetParentCube) + "_" + str(i)
                self.joints.append(JointName)
                pyrosim.Send_Joint(name=JointName, parent=str(targetParentCube), child=str(i), type="revolute",
                                   position=pos, jointAxis=jA)
                # joint gotten, now cube is called down
                # must call down cube to ensure that it is in a position appropriate given the nextOccupied area
                # complicated, we try this naive solution for the moment.
                # we need to figure out how to create the spacing given
                relativePos = self.createPositionalArgument(nextOccupied, [sizeX, sizeY, sizeZ])
                if i in self.sensorBlocks:
                    self.blocks.append(BodyCube(name=str(i), position=relativePos, size=[sizeX, sizeY, sizeZ],
                                                s1='<material name="Green">', s2='    <color rgba="0 1.0 0.0 1.0"/>'))
                else:
                    self.blocks.append(BodyCube(name=str(i), position=relativePos, size=[sizeX, sizeY, sizeZ],
                                                s1='<material name="Cyan">', s2='    <color rgba="0 1.0 1.0 1.0"/>'))
                # tricky, if we block off the -x of previous, we want to block the +x of this block
                # how do we do this?
                # if nextOccupied is odd -> attributes to 1, 3, 5 -> 0, 2, 4 respectively
                # if nextOccupied is even -> attributes to 0, 2, 4 -> 1, 3, 5. this line should do:
                self.blocks[i].setOccupied(nextOccupied - 1 if nextOccupied % 2 == 1 else nextOccupied + 1)
        pyrosim.End()

    def createJointArgument(self, currSize, nextSize, nextOccupied):
        if nextOccupied == 0:
            # generate block in +x
            position = [currSize[0]/2 - nextSize[0]/2, 0, 0]
            JointAxis = "0 1 1"
        elif nextOccupied == 1:
            # generate block in -x
            position = [-currSize[0]/2 + nextSize[0]/2, 0, 0]
            JointAxis = "0 1 1"
        elif nextOccupied == 2:
            # generate block in +y
            position = [0, currSize[1]/2 - nextSize[1]/2, 0]
            JointAxis = "1 0 1"
        elif nextOccupied == 3:
            # generate block in -y
            position = [0, -currSize[1]/2 + nextSize[1]/2, 0]
            JointAxis = "1 0 1"
        elif nextOccupied == 4:
            # generate block in +z
            position = [0, 0, currSize[2]/2 - nextSize[0]/2]
            JointAxis = "1 1 0"
        elif nextOccupied == 5:
            # generate block in -z
            position = [0, 0, -currSize[2]/2 + nextSize[0]/2]
            JointAxis = "1 1 0"
        else:
            # generate block in + x
            print("what")
            position = [0, 0, 0]
            JointAxis = "0 0 1"
        return position, JointAxis

    def createPositionalArgument(self, facing, size):
        if facing == 0:
            return [size[0], 0, 0]
        if facing == 1:
            return [-size[0], 0, 0]
        if facing == 2:
            return [0, size[1], 0]
        if facing == 3:
            return [0, -size[1], 0]
        if facing == 4:
            return [0, 0, size[2]]
        else:
            return [0, 0, -size[2]]

    def Create_Brain(self, segmentCount):
        pyrosim.Start_NeuralNetwork("brain" + str(self.ID) + ".nndf")
        # let's set a minimum probability
        motorProbability = random.random()
        while motorProbability < .7:
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
        for i in range(len(self.joints)):
            # only send a sensor or a motor if we roll lower than the set probability
            if random.random() < motorProbability:
                # add the sensor
                pyrosim.Send_Motor_Neuron(name=sensorCount + motorCount, jointName=self.joints[i])
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
