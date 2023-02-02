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
        self.weights = self.weights * 2 - 1
        self.fitness = math.inf
        self.ID = ID
        # print(self.weights)

    def Set_ID(self, ID):
        self.ID = ID

    def Start_Simulation(self, DoG):
        self.Create_World()
        self.Create_Body()
        self.Create_Brain()
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
        randRow = random.randint(0, c.numSensorNeurons - 1)
        randCol = random.randint(0, c.numMotorNeurons - 1)
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
        pyrosim.Send_Cube(name="Box", pos=[x, y, z], size=[length, width, height])
        pyrosim.End()

    def Create_Body(self):
        # length = 1
        # width = 1
        # height = 1
        # x = 0
        # y = 0
        # z = 1
        legsize = [.15, .15, .3]
        pyrosim.Start_URDF("body.urdf")
        pyrosim.Send_Cube(name="Torso", pos=[0, 0, .6], size=[3, 1, .3])
        pyrosim.Send_Joint(name="Torso_BottomCrest", parent="Torso", child="BottomCrest", type="revolute",
                           position=[0, 0, 0.375], jointAxis="1 0 0")
        pyrosim.Send_Cube(name="BottomCrest", pos=[0, 0, 0], size=[3, .2, .15])

        pyrosim.Send_Joint(name="Torso_L1", parent="Torso", child="L1", type="revolute",
                           position=[0, 0.45, 0.3], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="L1", pos=[0, 0, 0], size=legsize)

        pyrosim.Send_Joint(name="Torso_R1", parent="Torso", child="R1", type="revolute",
                           position=[0, -0.45, 0.3], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="R1", pos=[0, 0, 0], size=legsize)

        pyrosim.Send_Joint(name="Torso_L2", parent="Torso", child="L2", type="revolute",
                           position=[0.5, 0.45, 0.3], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="L2", pos=[0, 0, 0], size=legsize)

        pyrosim.Send_Joint(name="Torso_R2", parent="Torso", child="R2", type="revolute",
                           position=[0.5, -0.45, 0.3], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="R2", pos=[0, 0, 0], size=legsize)

        pyrosim.Send_Joint(name="Torso_L3", parent="Torso", child="L3", type="revolute",
                           position=[1, 0.45, 0.3], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="L3", pos=[0, 0, 0], size=legsize)

        pyrosim.Send_Joint(name="Torso_R3", parent="Torso", child="R3", type="revolute",
                           position=[1, -0.45, 0.3], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="R3", pos=[0, 0, 0], size=legsize)

        pyrosim.Send_Joint(name="Torso_L4", parent="Torso", child="L4", type="revolute",
                           position=[1.5, 0.45, 0.3], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="L4", pos=[0, 0, 0], size=legsize)

        pyrosim.Send_Joint(name="Torso_R4", parent="Torso", child="R4", type="revolute",
                           position=[1.5, -0.45, 0.3], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="R4", pos=[0, 0, 0], size=legsize)

        # negatives begin here

        pyrosim.Send_Joint(name="Torso_L5", parent="Torso", child="L5", type="revolute",
                           position=[-1.5, 0.45, 0.3], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="L5", pos=[0, 0, 0], size=legsize)

        pyrosim.Send_Joint(name="Torso_R5", parent="Torso", child="R5", type="revolute",
                           position=[-1.5, -0.45, 0.3], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="R5", pos=[0, 0, 0], size=legsize)

        pyrosim.Send_Joint(name="Torso_L6", parent="Torso", child="L6", type="revolute",
                           position=[-1.0, 0.45, 0.3], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="L6", pos=[0, 0, 0], size=legsize)

        pyrosim.Send_Joint(name="Torso_R6", parent="Torso", child="R6", type="revolute",
                           position=[-1.0, -0.45, 0.3], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="R6", pos=[0, 0, 0], size=legsize)

        pyrosim.Send_Joint(name="Torso_L7", parent="Torso", child="L7", type="revolute",
                           position=[-0.5, 0.45, 0.3], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="L7", pos=[0, 0, 0], size=legsize)

        pyrosim.Send_Joint(name="Torso_R7", parent="Torso", child="R7", type="revolute",
                           position=[-0.5, -0.45, 0.3], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="R7", pos=[0, 0, 0], size=legsize)

        # pyrosim.Send_Cube(name="Torso", pos=[x, y, z], size=[length, width, height])
        #
        # # backleg
        # pyrosim.Send_Joint(name="Torso_BackLeg", parent="Torso", child="BackLeg", type="revolute",
        #                    position=[0, -0.5, 1.0], jointAxis="1 0 0")
        # pyrosim.Send_Cube(name="BackLeg", pos=[0, -.5, 0], size=[.2, 1, .2])
        #
        # pyrosim.Send_Joint(name="BackLeg_BackLower", parent="BackLeg", child="BackLower", type="revolute",
        #                    position=[0, -1, 0], jointAxis="0 1 1")
        # pyrosim.Send_Cube(name="BackLower", pos=[0, 0, -.5], size=[.2, .2, 1])
        #
        # # frontleg
        # pyrosim.Send_Joint(name="Torso_FrontLeg", parent="Torso", child="FrontLeg", type="revolute",
        #                    position=[0, 0.5, 1.0], jointAxis="1 0 0")
        # pyrosim.Send_Cube(name="FrontLeg", pos=[0, 0.5, 0], size=[.2, 1, .2])
        # pyrosim.Send_Joint(name="FrontLeg_FrontLower", parent="FrontLeg", child="FrontLower", type="revolute",
        #                    position=[0, 1, 0], jointAxis="0 1 1")
        # pyrosim.Send_Cube(name="FrontLower", pos=[0, 0, -.5], size=[.2, .2, 1])
        #
        # # leftleg
        # pyrosim.Send_Joint(name="Torso_LeftLeg", parent="Torso", child="LeftLeg", type="revolute",
        #                    position=[0.5, 0, 1.0], jointAxis="0 1 0")
        # pyrosim.Send_Cube(name="LeftLeg", pos=[0.5, 0, 0], size=[1, .2, .2])
        # pyrosim.Send_Joint(name="LeftLeg_LeftLower", parent="LeftLeg", child="LeftLower", type="revolute",
        #                    position=[1, 0, 0], jointAxis="0 1 1")
        # pyrosim.Send_Cube(name="LeftLower", pos=[0, 0, -.5], size=[.2, .2, 1])
        #
        # # rightleg
        # pyrosim.Send_Joint(name="Torso_RightLeg", parent="Torso", child="RightLeg", type="revolute",
        #                    position=[-0.5, 0, 1.0], jointAxis="0 1 0")
        # pyrosim.Send_Cube(name="RightLeg", pos=[-.5, 0, 0], size=[1, .2, .2])
        # pyrosim.Send_Joint(name="RightLeg_RightLower", parent="RightLeg", child="RightLower", type="revolute",
        #                    position=[-1, 0, 0], jointAxis="0 1 1")
        # pyrosim.Send_Cube(name="RightLower", pos=[0, 0, -.5], size=[.2, .2, 1])

        pyrosim.End()

    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.ID) + ".nndf")
        pyrosim.Send_Sensor_Neuron(name=0, linkName="Torso")
        pyrosim.Send_Sensor_Neuron(name=1, linkName="BottomCrest")
        pyrosim.Send_Sensor_Neuron(name=2, linkName="L1")
        pyrosim.Send_Sensor_Neuron(name=3, linkName="R1")
        pyrosim.Send_Sensor_Neuron(name=4, linkName="L2")
        pyrosim.Send_Sensor_Neuron(name=5, linkName="R2")
        pyrosim.Send_Sensor_Neuron(name=6, linkName="L3")
        pyrosim.Send_Sensor_Neuron(name=7, linkName="R3")
        pyrosim.Send_Sensor_Neuron(name=8, linkName="L4")
        pyrosim.Send_Sensor_Neuron(name=9, linkName="R4")
        pyrosim.Send_Sensor_Neuron(name=10, linkName="L5")
        pyrosim.Send_Sensor_Neuron(name=11, linkName="R5")
        pyrosim.Send_Sensor_Neuron(name=12, linkName="L6")
        pyrosim.Send_Sensor_Neuron(name=13, linkName="R6")
        pyrosim.Send_Sensor_Neuron(name=14, linkName="L7")
        pyrosim.Send_Sensor_Neuron(name=15, linkName="R7")
        pyrosim.Send_Motor_Neuron(name=16, jointName="Torso_L1")
        pyrosim.Send_Motor_Neuron(name=17, jointName="Torso_R1")
        pyrosim.Send_Motor_Neuron(name=18, jointName="Torso_L2")
        pyrosim.Send_Motor_Neuron(name=19, jointName="Torso_R2")
        pyrosim.Send_Motor_Neuron(name=20, jointName="Torso_L3")
        pyrosim.Send_Motor_Neuron(name=21, jointName="Torso_R3")
        pyrosim.Send_Motor_Neuron(name=22, jointName="Torso_L4")
        pyrosim.Send_Motor_Neuron(name=23, jointName="Torso_R4")
        pyrosim.Send_Motor_Neuron(name=24, jointName="Torso_L5")
        pyrosim.Send_Motor_Neuron(name=25, jointName="Torso_R5")
        pyrosim.Send_Motor_Neuron(name=26, jointName="Torso_L6")
        pyrosim.Send_Motor_Neuron(name=27, jointName="Torso_R6")
        pyrosim.Send_Motor_Neuron(name=28, jointName="Torso_L7")
        pyrosim.Send_Motor_Neuron(name=29, jointName="Torso_R7")
        # pyrosim.Send_Sensor_Neuron(name=0, linkName="Torso")
        # pyrosim.Send_Sensor_Neuron(name=1, linkName="BackLeg")
        # pyrosim.Send_Sensor_Neuron(name=2, linkName="FrontLeg")
        # pyrosim.Send_Sensor_Neuron(name=3, linkName="LeftLeg")
        # pyrosim.Send_Sensor_Neuron(name=4, linkName="RightLeg")
        # pyrosim.Send_Sensor_Neuron(name=5, linkName="BackLower")
        # pyrosim.Send_Sensor_Neuron(name=6, linkName="FrontLower")
        # pyrosim.Send_Sensor_Neuron(name=7, linkName="LeftLower")
        # pyrosim.Send_Sensor_Neuron(name=8, linkName="RightLower")
        # pyrosim.Send_Motor_Neuron(name=9, jointName="Torso_BackLeg")
        # pyrosim.Send_Motor_Neuron(name=10, jointName="Torso_FrontLeg")
        # pyrosim.Send_Motor_Neuron(name=11, jointName="Torso_LeftLeg")
        # pyrosim.Send_Motor_Neuron(name=12, jointName="Torso_RightLeg")
        # pyrosim.Send_Motor_Neuron(name=13, jointName="BackLeg_BackLower")
        # pyrosim.Send_Motor_Neuron(name=14, jointName="FrontLeg_FrontLower")
        # pyrosim.Send_Motor_Neuron(name=15, jointName="LeftLeg_LeftLower")
        # pyrosim.Send_Motor_Neuron(name=16, jointName="RightLeg_RightLower")
        for i in range(c.numSensorNeurons):  # current row
            for j in range(c.numMotorNeurons):  # current column
                pyrosim.Send_Synapse(sourceNeuronName=i, targetNeuronName=j + c.numSensorNeurons,
                                     weight=self.weights[i][j])

        pyrosim.End()
