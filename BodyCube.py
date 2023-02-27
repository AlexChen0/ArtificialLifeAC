import pyrosim.pyrosim as pyrosim


class BodyCube:
    def __init__(self, name, position, size, relPos, parentJointPos, parentJointFace, s1, s2):
        # we always need to make a joint, so this is no issue
        self.name = name
        self.position = position
        self.size = size
        # occupied: [+x, -x, +y, -y, +z, -z]
        self.relPosition = relPos
        self.parentJointPos = parentJointPos
        self.parentJointFace = parentJointFace
        self.occupied = [0, 0, 0, 0, 0, 0]
        self.s1 = s1
        self.s2 = s2

    def setOccupied(self, target):
        self.occupied[target] = 1

    def setUnoccupied(self, target):
        self.occupied[target] = 0

    def setAllUnoccupied(self):
        self.occupied = [0, 0, 0, 0, 0, 0]

    def setRelPos(self, value):
        self.relPosition = value

    def setParentJointPos(self, value):
        self.parentJointPos = value

    def setParentJointFace(self, value):
        self.parentJointFace = value

    def setPosition(self, value):
        self.position = value
