import numpy as np

ampFront = np.pi/4
freqFront = 10
phaseOffsetFront = 0
ampBack = np.pi/4
freqBack = 10
phaseOffsetBack = np.pi/2
gravity = -9.8
numIterations =1500
frameRate = 1/300
maxForce = 200
numGenerations = 20 # remember these numbers are going to 150, 20 for the final proj -> run on desktop
populationSize = 5
numSensorNeurons = 100
numMotorNeurons = 100
motorJointRange = 1
