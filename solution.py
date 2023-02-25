import math
import numpy as np
import pyrosim.pyrosim as pyrosim
import os
import random
import time
import constants as c
from BodyCube import BodyCube
from JointArg import jointArg


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
        self.relPositions = []
        self.jointArgs = []
        self.motorJoints = []
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
        pyrosim.Send_Cube(name="Box", pos=[x, y, z], size=[length, width, height], s1='<material name="Cyan">',
                          s2='    <color rgba="0 1.0 1.0 1.0"/>')
        pyrosim.End()

    def Create_Body(self):
        # completely new body assembled if there is no parameter
        self.numSegments = random.randint(8, 15)
        sensorProbability = .65
        self.sensorBlocks = []
        self.joints = []
        self.blocks = []
        self.relPositions = []
        self.jointArgs = []
        for i in range(self.numSegments):
            if random.random() < sensorProbability:
                self.sensorBlocks.append(i)
        self.numSensors = len(self.sensorBlocks)
        # print("numSegments: ", self.numSegments)
        pyrosim.Start_URDF("body" + str(self.ID) + ".urdf")
        # dummy nexts used to figure out positional garbage
        for i in range(self.numSegments):
            sizeX = random.random() * .8 + .2
            sizeY = random.random() * .8 + .2
            sizeZ = random.random() * .8 + .2
            # sizeX = 1
            # sizeY = 1
            # sizeZ = 1
            if i == 0:
                # first.
                # create initial sizes and get next sizes
                if i in self.sensorBlocks:
                    self.relPositions.append([0, 0, 0])
                    self.blocks.append(BodyCube(name=str(i), position=[0, 0, 2 + sizeZ / 2], size=[sizeX, sizeY, sizeZ],
                                                relPos=[0, 0, 0], parentJointPos=[0, 0, 2], parentJointFace=6,
                                                s1='<material name="Green">', s2='    <color rgba="0 1.0 0.0 1.0"/>'))
                    pyrosim.Send_Cube(name=str(i), pos=[0, 0, 2 + sizeZ / 2], size=[sizeX, sizeY, sizeZ],
                                      s1='<material name="Green">', s2='    <color rgba="0 1.0 0.0 1.0"/>')
                else:
                    self.relPositions.append([0, 0, 0])
                    self.blocks.append(BodyCube(name=str(i), position=[0, 0, 2 + sizeZ / 2], size=[sizeX, sizeY, sizeZ],
                                                relPos=[0, 0, 0], parentJointPos=[0, 0, 2], parentJointFace=6,
                                                s1='<material name="Cyan">', s2='    <color rgba="0 1.0 1.0 1.0"/>'))
                    pyrosim.Send_Cube(name=str(i), pos=[0, 0, 2 + sizeZ / 2], size=[sizeX, sizeY, sizeZ],
                                      s1='<material name="Cyan">', s2='    <color rgba="0 1.0 1.0 1.0"/>')
            else:
                # in the middle and for the last
                # position should be 0 0 0 thanks to joint stuff already figured out
                # that said we need to figure out from the next target block where to assemble this one
                # joints do not need to be tracked by BodyCube, since they are guaranteed between 2 cubes
                # that said, now we need to decide which is the next segment that will be occupied
                # therefore, we no longer calculate where the joint is here, and we do it before calling cubes
                # but watch out: we need to figure out where the next cube goes too -- who's the parent?
                while True:
                    targetParentCube = random.randint(0, len(self.blocks) - 1)
                    # now we can check the occupation
                    if self.blocks[targetParentCube].occupied != [1, 1, 1, 1, 1, 1]:
                        openIndicies = [i for i in range(len(self.blocks[targetParentCube].occupied))
                                        if self.blocks[targetParentCube].occupied[i] == 0]
                        for j in openIndicies:
                            # we need to now see if the index has what number,
                            # and then compare to prev blocks relative position
                            relPosCheck = self.calculateRelativePosition(targetParentCube, j)
                            if relPosCheck in self.relPositions:
                                openIndicies.remove(j)
                            if relPosCheck[2] < -3:
                                openIndicies.remove(j)
                        # now if there's still possible areas to add links, we accept this block
                        if len(openIndicies) > 0:
                            break
                nextOccupied = random.choice(openIndicies)
                # need to calc new relative position
                newRelPos = self.calculateRelativePosition(targetParentCube, nextOccupied)
                pos, jA = self.createJointArgument([sizeX, sizeY, sizeZ], targetParentCube,
                                                   self.blocks[targetParentCube].size,
                                                   self.blocks[targetParentCube].parentJointFace, nextOccupied)
                # where the joint is is decided by nextOccupied, since we are either going to be making it in one of the
                # 6 faces of the block, now we set that to be occupied
                self.blocks[targetParentCube].setOccupied(nextOccupied)
                JointName = str(targetParentCube) + "_" + str(i)
                self.joints.append(JointName)
                pyrosim.Send_Joint(name=JointName, parent=str(targetParentCube), child=str(i), type="revolute",
                                   position=pos, jointAxis=jA)
                self.jointArgs.append(jointArg(name=JointName, parent=str(targetParentCube),
                                               child=str(i), position=pos, jointAxis=jA))
                # joint gotten, now cube is called down
                # must call down cube to ensure that it is in a position appropriate given the nextOccupied area
                # complicated, we try this naive solution for the moment.
                # we need to figure out how to create the spacing given
                relativePos = self.createPositionalArgument(nextOccupied, [sizeX, sizeY, sizeZ])
                nextJointFace = nextOccupied - 1 if nextOccupied % 2 == 1 else nextOccupied + 1
                if i in self.sensorBlocks:
                    self.blocks.append(BodyCube(name=str(i), position=relativePos, size=[sizeX, sizeY, sizeZ],
                                                relPos=newRelPos, parentJointPos=pos, parentJointFace=nextJointFace,
                                                s1='<material name="Green">', s2='    <color rgba="0 1.0 0.0 1.0"/>'))
                    pyrosim.Send_Cube(name=str(i), pos=relativePos, size=[sizeX, sizeY, sizeZ],
                                      s1='<material name="Green">', s2='    <color rgba="0 1.0 0.0 1.0"/>')
                else:
                    self.blocks.append(BodyCube(name=str(i), position=relativePos, size=[sizeX, sizeY, sizeZ],
                                                relPos=newRelPos, parentJointPos=pos, parentJointFace=nextJointFace,
                                                s1='<material name="Cyan">', s2='    <color rgba="0 1.0 1.0 1.0"/>'))
                    pyrosim.Send_Cube(name=str(i), pos=relativePos, size=[sizeX, sizeY, sizeZ],
                                      s1='<material name="Cyan">', s2='    <color rgba="0 1.0 1.0 1.0"/>')
                # tricky, if we block off the -x of previous, we want to block the +x of this block
                # how do we do this?
                # if nextOccupied is odd -> attributes to 1, 3, 5 -> 0, 2, 4 respectively
                # if nextOccupied is even -> attributes to 0, 2, 4 -> 1, 3, 5. this line should do:
                self.blocks[i].setOccupied(nextOccupied - 1 if nextOccupied % 2 == 1 else nextOccupied + 1)
        pyrosim.End()
        # print(self.joints)

    def Create_Body_From_Existing(self, generationCount):
        # creates the body out of the existing joint and link subsets, so it loads the data which can then be used
        # as basis for the body and brain
        pyrosim.Start_URDF("body" + str(self.ID) + ".urdf")
        for i in range(len(self.blocks)):
            if i == 0:
                self.relPositions.append([0, 0, 0])
                pyrosim.Send_Cube(name=self.blocks[i].name, pos=self.blocks[i].position, size=self.blocks[i].size,
                                  s1=self.blocks[i].s1, s2=self.blocks[i].s2)
            else:
                # we can assume that the joints and blocks are all in order, so we can do this
                pyrosim.Send_Joint(name=self.jointArgs[i-1].name, parent=self.jointArgs[i-1].parent,
                                   child=self.jointArgs[i-1].child, type=self.jointArgs[i-1].theType,
                                   position=self.jointArgs[i-1].position, jointAxis=self.jointArgs[i-1].jointAxis)
                pyrosim.Send_Cube(name=self.blocks[i].name, pos=self.blocks[i].position, size=self.blocks[i].size,
                                  s1=self.blocks[i].s1, s2=self.blocks[i].s2)
            pyrosim.End()

    def createJointArgument(self, currSize, index, parentBlockSize, parentJointFace, nextOccupied):
        if index == 0:
            # absolute position
            if nextOccupied == 0:
                # generate block in +x
                position = [currSize[0] / 2, 0, 2 + currSize[2] / 2]
                JointAxis = "0 1 1"
            elif nextOccupied == 1:
                # generate block in -x
                position = [-currSize[0] / 2, 0, 2 + currSize[2] / 2]
                JointAxis = "0 1 1"
            elif nextOccupied == 2:
                # generate block in +y
                position = [0, currSize[1] / 2, 2 + currSize[2] / 2]
                JointAxis = "1 0 1"
            elif nextOccupied == 3:
                # generate block in -y
                position = [0, -currSize[1] / 2, 2 + currSize[2] / 2]
                JointAxis = "1 0 1"
            elif nextOccupied == 4:
                # generate block in +z
                position = [0, 0, 2 + currSize[2]]
                JointAxis = "1 1 0"
            elif nextOccupied == 5:
                # generate block in -z
                position = [0, 0, 2]
                JointAxis = "1 1 0"
            else:
                # generate block in + x
                print("what")
                position = [0, 0, 0]
                JointAxis = "0 0 1"
        else:
            if parentJointFace == 1:
                if nextOccupied == 0:
                    # generate block in +x
                    position = [parentBlockSize[0], 0, 0]
                    JointAxis = "0 1 1"
                elif nextOccupied == 1:
                    # generate block in -x
                    print("!!!! WARNING: INSTANTIATED IN IMPOSSIBLE SPACE")
                    print("debug: parent +x child -x")
                    position = [0, 0, 0]
                    JointAxis = "0 0 1"
                elif nextOccupied == 2:
                    # generate block in +y
                    position = [parentBlockSize[0] / 2, parentBlockSize[1] / 2, 0]
                    JointAxis = "1 0 1"
                elif nextOccupied == 3:
                    # generate block in -y
                    position = [parentBlockSize[0] / 2, -parentBlockSize[1] / 2, 0]
                    JointAxis = "1 0 1"
                elif nextOccupied == 4:
                    # generate block in +z
                    position = [parentBlockSize[0] / 2, 0, parentBlockSize[2] / 2]
                    JointAxis = "1 1 0"
                elif nextOccupied == 5:
                    # generate block in -z
                    position = [parentBlockSize[0] / 2, 0, -parentBlockSize[2] / 2]
                    JointAxis = "1 1 0"
                else:
                    # generate block in + x
                    print("what")
                    position = [0, 0, 0]
                    JointAxis = "0 0 1"
            elif parentJointFace == 0:
                if nextOccupied == 0:
                    # generate block in +x
                    print("!!!! WARNING: INSTANTIATED IN IMPOSSIBLE SPACE")
                    print("debug: parent -x child +x")
                    position = [0, 0, 0]
                    JointAxis = "0 0 1"
                elif nextOccupied == 1:
                    # generate block in -x
                    position = [-parentBlockSize[0], 0, 0]
                    JointAxis = "0 1 1"
                elif nextOccupied == 2:
                    # generate block in +y
                    position = [-parentBlockSize[0] / 2, parentBlockSize[1] / 2, 0]
                    JointAxis = "1 0 1"
                elif nextOccupied == 3:
                    # generate block in -y
                    position = [-parentBlockSize[0] / 2, -parentBlockSize[1] / 2, 0]
                    JointAxis = "1 0 1"
                elif nextOccupied == 4:
                    # generate block in +z
                    position = [-parentBlockSize[0] / 2, 0, parentBlockSize[2] / 2]
                    JointAxis = "1 1 0"
                elif nextOccupied == 5:
                    # generate block in -z
                    position = [-parentBlockSize[0] / 2, 0, -parentBlockSize[2] / 2]
                    JointAxis = "1 1 0"
                else:
                    # generate block in + x
                    print("what")
                    position = [0, 0, 0]
                    JointAxis = "0 0 1"
            elif parentJointFace == 3:
                if nextOccupied == 0:
                    # generate block in +x
                    position = [parentBlockSize[0] / 2, parentBlockSize[1] / 2, 0]
                    JointAxis = "0 1 1"
                elif nextOccupied == 1:
                    # generate block in -x
                    position = [-parentBlockSize[0] / 2, parentBlockSize[1] / 2, 0]
                    JointAxis = "0 1 1"
                elif nextOccupied == 2:
                    # generate block in +y
                    position = [0, parentBlockSize[1], 0]
                    JointAxis = "1 0 1"
                elif nextOccupied == 3:
                    # generate block in -y
                    print("!!!! WARNING: INSTANTIATED IN IMPOSSIBLE SPACE")
                    print("debug: parent +y child -y")
                    position = [0, 0, 0]
                    JointAxis = "0 0 1"
                elif nextOccupied == 4:
                    # generate block in +z
                    position = [0, parentBlockSize[1] / 2, parentBlockSize[2] / 2]
                    JointAxis = "1 1 0"
                elif nextOccupied == 5:
                    # generate block in -z
                    position = [0, parentBlockSize[1] / 2, -parentBlockSize[2] / 2]
                    JointAxis = "1 1 0"
                else:
                    # generate block in + x
                    print("what")
                    position = [0, 0, 0]
                    JointAxis = "0 0 1"
            elif parentJointFace == 2:
                if nextOccupied == 0:
                    # generate block in +x
                    position = [parentBlockSize[0] / 2, -parentBlockSize[1] / 2, 0]
                    JointAxis = "0 1 1"
                elif nextOccupied == 1:
                    # generate block in -x
                    position = [-parentBlockSize[0], -parentBlockSize[1] / 2, 0]
                    JointAxis = "0 1 1"
                elif nextOccupied == 2:
                    # generate block in +y
                    print("!!!! WARNING: INSTANTIATED IN IMPOSSIBLE SPACE")
                    print("debug: parent -y child +y")
                    position = [0, 0, 0]
                    JointAxis = "0 0 1"
                elif nextOccupied == 3:
                    # generate block in -y
                    position = [0, -parentBlockSize[1], 0]
                    JointAxis = "1 0 1"
                elif nextOccupied == 4:
                    # generate block in +z
                    position = [0, -parentBlockSize[1] / 2, parentBlockSize[2] / 2]
                    JointAxis = "1 1 0"
                elif nextOccupied == 5:
                    # generate block in -z
                    position = [0, -parentBlockSize[1] / 2, -parentBlockSize[2] / 2]
                    JointAxis = "1 1 0"
                else:
                    # generate block in + x
                    print("what")
                    position = [0, 0, 0]
                    JointAxis = "0 0 1"
            elif parentJointFace == 5:
                if nextOccupied == 0:
                    # generate block in +x
                    position = [parentBlockSize[0] / 2, 0, parentBlockSize[2] / 2]
                    JointAxis = "0 1 1"
                elif nextOccupied == 1:
                    # generate block in -x
                    position = [-parentBlockSize[0] / 2, 0, parentBlockSize[2] / 2]
                    JointAxis = "0 1 1"
                elif nextOccupied == 2:
                    # generate block in +y
                    position = [0, parentBlockSize[1] / 2, parentBlockSize[2] / 2]
                    JointAxis = "1 0 1"
                elif nextOccupied == 3:
                    # generate block in -y
                    position = [0, -parentBlockSize[1] / 2, parentBlockSize[2] / 2]
                    JointAxis = "1 0 1"
                elif nextOccupied == 4:
                    # generate block in +z
                    position = [0, 0, parentBlockSize[2]]
                    JointAxis = "1 1 0"
                elif nextOccupied == 5:
                    # generate block in -z
                    print("!!!! WARNING: INSTANTIATED IN IMPOSSIBLE SPACE")
                    print("debug: parent +z child -z")
                    position = [0, 0, 1]
                    JointAxis = "0 0 0"
                else:
                    # generate block in + x
                    print("what")
                    position = [0, 0, 0]
                    JointAxis = "0 0 1"
            elif parentJointFace == 4:
                if nextOccupied == 0:
                    # generate block in +x
                    position = [parentBlockSize[0] / 2, 0, -parentBlockSize[2] / 2]
                    JointAxis = "0 1 1"
                elif nextOccupied == 1:
                    # generate block in -x
                    position = [-parentBlockSize[0] / 2, 0, -parentBlockSize[2] / 2]
                    JointAxis = "0 1 1"
                elif nextOccupied == 2:
                    # generate block in +y
                    position = [0, parentBlockSize[1] / 2, -parentBlockSize[2] / 2]
                    JointAxis = "1 0 1"
                elif nextOccupied == 3:
                    # generate block in -y
                    position = [0, -parentBlockSize[1] / 2, -parentBlockSize[2] / 2]
                    JointAxis = "1 0 1"
                elif nextOccupied == 4:
                    # generate block in +z
                    print("!!!! WARNING: INSTANTIATED IN IMPOSSIBLE SPACE")
                    print("debug: parent -z child +z")
                    position = [0, 0, 1]
                    JointAxis = "0 0 0"
                elif nextOccupied == 5:
                    # generate block in -z
                    position = [0, 0, -parentBlockSize[2]]
                    JointAxis = "1 1 0"
                else:
                    # generate block in + x
                    print("what")
                    position = [0, 0, 0]
                    JointAxis = "0 0 1"
            else:
                position = [0, 0, 0]
                JointAxis = "0 0 1"
        return position, JointAxis

    def calculateRelativePosition(self, parentCubeIndex, index):
        relPosCheck = self.blocks[parentCubeIndex].relPosition
        if index == 0:
            # +x
            relPosCheck[0] += 1
        elif index == 1:
            relPosCheck[0] -= 1
        elif index == 2:
            relPosCheck[1] += 1
        elif index == 3:
            relPosCheck[1] -= 1
        elif index == 4:
            relPosCheck[2] += 1
        else:
            relPosCheck[2] -= 1
        return relPosCheck

    def createPositionalArgument(self, facing, size):
        if facing == 0:
            return [size[0] / 2, 0, 0]
        if facing == 1:
            return [-size[0] / 2, 0, 0]
        if facing == 2:
            return [0, size[1] / 2, 0]
        if facing == 3:
            return [0, -size[1] / 2, 0]
        if facing == 4:
            return [0, 0, size[2] / 2]
        else:
            return [0, 0, -size[2] / 2]

    def Create_Brain(self, segmentCount):
        self.motorJoints = []
        pyrosim.Start_NeuralNetwork("brain" + str(self.ID) + ".nndf")
        # let's set a minimum probability
        motorProbability = random.random()
        while motorProbability < .6:
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
                self.motorJoints.append(self.joints[i])
                motorCount += 1
        # set self.weights here now
        self.numMotors = motorCount
        self.weights = np.random.rand(sensorCount, motorCount)
        self.weights = self.weights * 2 - 1
        for i in range(sensorCount):  # current row
            for j in range(motorCount):  # current column
                pyrosim.Send_Synapse(sourceNeuronName=i, targetNeuronName=j + sensorCount,
                                     weight=self.weights[i][j])
        # print("sensor count:", self.numSensors)
        # print("motor count:", self.numMotors)
        pyrosim.End()

    def Create_Brain_From_Existing(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.ID) + ".nndf")
        sensorCount = 0
        motorCount = 0
        for i in self.sensorBlocks:
            pyrosim.Send_Sensor_Neuron(name=sensorCount, linkName=str(i))
            sensorCount += 1
        for i in self.motorJoints:
            pyrosim.Send_Motor_Neuron(name=sensorCount + motorCount, jointName=i)
            motorCount += 1

        for i in range(sensorCount):  # current row
            for j in range(motorCount):  # current column
                pyrosim.Send_Synapse(sourceNeuronName=i, targetNeuronName=j + sensorCount,
                                     weight=self.weights[i][j])
        pyrosim.End()
