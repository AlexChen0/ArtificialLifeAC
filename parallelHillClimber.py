import math

import solution
import constants as c
import copy
import os
import matplotlib.pyplot as plt


class PARALLEL_HILL_CLIMBER:
    def __init__(self):
        os.system("rm brain*.nndf")
        os.system("rm fitness*.txt")
        self.parents = {}
        self.children = {}
        self.nextID = 0
        self.graphVals = {}
        for i in range(c.populationSize):
            self.parents[i] = solution.SOLUTION(self.nextID)
            self.nextID += 1
            self.graphVals[i] = []

    # To the instructor and creator of ludobots, if you're reading this, your ludobots
    # instructions are fucking garbage.
    def Evolve(self):
        if self.nextID == 0:
            self.Evaluate(self.parents, "GUI")
        else:
            self.Evaluate(self.parents, "DIRECT")
        for j in range(c.numGenerations):
            # self.parent.Evaluate("DIRECT")
            self.Evolve_For_One_Generation(j)

    def Evolve_For_One_Generation(self, generation):
        self.Spawn()
        self.Mutate()
        if generation == 0:
            self.Evaluate(self.children, "DIRECT")
        else:
            self.Evaluate(self.children, "DIRECT")
        # self.Print()
        print("new generation count: ", generation)
        self.Select()

    def Spawn(self):
        for i in self.parents.keys():
            self.children[i] = copy.deepcopy(self.parents[i])
            self.children[i].Set_ID(self.nextID)
            self.nextID += 1

    def Mutate(self):
        for i in self.children.keys():
            self.children[i].Mutate()

    def Evaluate(self, solutions, typeOfSim):
        for i in range(c.populationSize):
            solutions[i].Start_Simulation(typeOfSim)
        for i in range(c.populationSize):
            solutions[i].Wait_For_Simulation_To_End()

    def Select(self):
        for i in self.parents.keys():
            if self.parents[i].fitness < self.children[i].fitness:
                self.parents[i] = self.children[i]
            self.graphVals[i].append(self.parents[i].fitness)

    def Show_Best(self):
        bestFitness = math.inf
        Best = None
        for i in self.parents.keys():
            if self.parents[i].fitness < bestFitness:
                bestFitness = self.parents[i].fitness
                Best = self.parents[i]
        Best.Start_Simulation("GUI")
        # this should save the best robot's body and brain
        Best.CreateBodyFromExisting()
        Best.CreateBrainFromExisting()
        x = [i for i in range(0, c.numGenerations)]
        for i in self.graphVals.keys():
            print(x, self.graphVals[i])
            plt.plot(x, self.graphVals[i], label="seed " + str(i + 1))
        plt.legend()
        plt.show()

    def Show_All(self):
        for i in self.parents.keys():
            self.parents[i].Start_Simulation("GUI")

    def Print(self):
        print()
        for i in self.parents.keys():
            print(self.parents[i].fitness, self.children[i].fitness)
        print()
