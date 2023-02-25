class jointArg:
    def __init__(self, name, parent, child, position, jointAxis, theType="revolute"):
        self.name = name
        self.parent = parent
        self.child = child
        self.theType = theType
        self.position = position
        self.jointAxis = jointAxis
