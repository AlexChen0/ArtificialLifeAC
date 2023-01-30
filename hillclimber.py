import solution
import constants as c
import copy

class HILL_CLIMBER:
    def __init__(self):
        self.parent = solution.SOLUTION()
        self.child = None

    # To the instructor and creator of ludobots, if you're reading this, your ludobots
    # instructions are fucking garbage.
    def Evolve(self):
        self.Show_Best()
        for i in range(c.numGenerations):
            self.parent.Evaluate("DIRECT")
            self.Evolve_For_One_Generation()

    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.child.Evaluate("DIRECT")
        self.Print()
        self.Select()

    def Spawn(self):
        self.child = copy.deepcopy(self.parent)

    def Mutate(self):
        self.child.Mutate()

    def Select(self):
        if self.parent.fitness > self.child.fitness:
            self.parent = self.child

    def Show_Best(self):
        self.parent.Evaluate("GUI")

    def Print(self):
        print(self.parent.fitness, self.child.fitness)

