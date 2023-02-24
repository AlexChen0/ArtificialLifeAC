import pybullet as p
import pyrosim.pyrosim as pyrosim
import time
import pybullet_data
import numpy as np
import random
import constants as c
import sys
from simulation import SIMULATION

# to remember: your reddit for this course is LudobotsIsBased
directOrGUI = sys.argv[1]
targetID = sys.argv[2]
print("targetID: ", targetID)
simulation = SIMULATION(directOrGUI, targetID)
simulation.run()

# frontPosData = np.sin((np.linspace(0, np.pi * 2, c.numIterations) * c.freqFront + c.phaseOffsetFront)) * c.ampFront

# backLegSensorValues = np.zeros(c.numIterations)
# frontLegSensorValues = np.zeros(c.numIterations)
simulation.Get_Fitness()

