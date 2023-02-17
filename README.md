# HOW THE CREATURES ARE GENERATED
At the start of generation, the number of boxes that will be generated is determined.
Afterwards, the program iterates through the boxes to decide which boxes will have sensors. 
Sensors are stored in self.sensors for ease of tracking and recollection. 

Once these boxes have been determined, the program first lays down the initial box, in the center of the plane. 
These boxes are kept track of within the BodyCube class, which stores position, size, and which faces of the cube are "open"

The remaining cubes are created in the following way: the program finds a cube that has an open face, selects one of the open faces on 
the cube, and "builds" a new cube on that face. The respective faces of the cubes are then marked as occupied in their separate BodyCube,
and a joint is created with regard to the relative position of the cube and the face being used. 

the joints are then stored in self.joints for ease of tracking and recollection

Once the last joint and cube are laid onto the body, the program ends creation of the body.

# HOW THE CREATURE BRAINS ARE GENERATED
Once the bodies have been generated, brains can now be generated to fit them. 

Since the number of segments are being kept track of, and we know the collection of which parts have sensors, we continue by sending
sensor neurons for each of the blocks that have been designated for sensors. 

Once the sensors are complete, motors can be generated for the joints that were created from the body. We look in self.joints and 
for each of the joints, we decide randomly which gets a motor placed. Finally, the array of sensors and motors get a designated value
chosen at random for the amount of weight assigned. Thus, the brain is complete. 

#DIAGRAM
A diagram can be found below, once I figure out how to do that. 
![alt text](https://github.com/AlexChen0/ArtificialLifeAC/blob/main/3DCreatureLogic.png)