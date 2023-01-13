import pyrosim.pyrosim as pyrosim

pyrosim.Start_SDF("world.sdf")
length = 1
width = 1
height = 1
x = 0
y = 0
z = 0.5
# for i in range(10):
#     pyrosim.Send_Cube(name="Box" + str(i), pos=[x, y, z + i], size=[length * (.9 ** i), width * (.9 ** i), height * (.9 ** i)])
# pyrosim.Send_Cube(name="Box2", pos=[x + 1,y,z + 1] , size=[length,width,height])
pyrosim.Send_Cube(name="Box", pos=[x,y,z] , size=[length,width,height])
pyrosim.End()

def create_World():
    length = 1
    width = 1
    height = 1
    x = -2
    y = 2
    z = 0.5
    pyrosim.Start_SDF("world.sdf")
    pyrosim.Send_Cube(name="Box", pos=[x, y, z], size=[length, width, height])
    pyrosim.End()

def create_Robot():
    length = 1
    width = 1
    height = 1
    x = 0
    y = 0
    z = 1.5
    pyrosim.Start_URDF("body.urdf")
    pyrosim.Send_Cube(name="Torso", pos=[x, y, z], size=[length, width, height])
    pyrosim.Send_Joint(name="Torso_BackLeg", parent="Torso", child="BackLeg", type="revolute", position=[-0.5, 0, 1.0])
    x = -0.5
    y = 0
    z = -0.5
    pyrosim.Send_Cube(name="BackLeg", pos=[x, y, z], size=[length, width, height])
    pyrosim.Send_Joint(name="Torso_FrontLeg", parent="Torso", child="FrontLeg", type="revolute", position=[0.5, 0, 1.0])
    x = 0.5
    y = 0
    z = -0.5
    pyrosim.Send_Cube(name="FrontLeg", pos=[x, y, z], size=[length, width, height])

    pyrosim.End()






create_World()
create_Robot()
