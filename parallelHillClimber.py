import math

import solution
import constants as c
import copy
import os


class PARALLEL_HILL_CLIMBER:
    def __init__(self):
        os.system("rm brain*.nndf")
        os.system("rm fitness*.txt")
        self.parents = {}
        self.children = {}
        self.nextID = 0
        for i in range(c.populationSize):
            self.parents[i] = solution.SOLUTION(self.nextID)
            self.nextID += 1

    # To the instructor and creator of ludobots, if you're reading this, your ludobots
    # instructions are fucking garbage.
    def Evolve(self):
        self.Evaluate(self.parents)
        for j in range(c.numGenerations):
            # self.parent.Evaluate("DIRECT")
            self.Evolve_For_One_Generation()

    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children)
        # self.Print()
        print("new generation")
        self.Select()

    def Spawn(self):
        for i in self.parents.keys():
            self.children[i] = copy.deepcopy(self.parents[i])
            self.children[i].Set_ID(self.nextID)
            self.nextID += 1

    def Mutate(self):
        for i in self.children.keys():
            self.children[i].Mutate()

    def Evaluate(self, solutions):
        for i in range(c.populationSize):
            solutions[i].Start_Simulation("GUI")
        for i in range(c.populationSize):
            solutions[i].Wait_For_Simulation_To_End()

    def Select(self):
        for i in self.parents.keys():
            if self.parents[i].fitness > self.children[i].fitness:
                self.parents[i] = self.children[i]

    def Show_Best(self):
        bestFitness = math.inf
        Best = None
        for i in self.parents.keys():
            if self.parents[i].fitness < bestFitness:
                bestFitness = self.parents[i].fitness
                Best = self.parents[i]
        Best.Start_Simulation("GUI")

    def Print(self):
        print()
        for i in self.parents.keys():
            print(self.parents[i].fitness, self.children[i].fitness)
        print()
