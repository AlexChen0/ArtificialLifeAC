# RUNNING THE PROGRAM
simply navigate to this directory and use the command:

python3 main.py

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

# HOW CREATURE BODIES ARE MODIFIED
Mutation can affect a body in one of two ways: addition of a link or deletion of a link. 

Addition of a link works identical to creature generation. Random cubes are selected until finally one is found with favorable spawning conditions (see diagram below).
Afterwards, the new cube is added, relevant data is saved for spawning purposes, and the simulation continues.

Deletion is more interesting. To do so, "leaf" joints/cubes are kept track of in order to only remove cubes that aren't parents to other cubes (as this would crash the program).
After a random leaf cube is selected, the corresponding joint is removed, and all relevant information about the cube is deleted. 

Moving one cube was originally going to be a feature, but that is equivalent to addition followed by deletion. This is thus deemed unnecessary.

# SOURCES
All information for setting up the bots was found in the reddit ludobots instructions starting here:

https://www.reddit.com/r/ludobots/wiki/installation/

# IMPROVING COLLISIONS
One of the main issues in previous editions of creating 3D creatues was many blocks having intersections with each other.
Therefore, I introduce the idea of "relative space" which is best looked at through induction. In our base case, we send in a first cube with a defined relative position of (0,0,0).

In our k+1 case, we look at the potential spaces we place our cube (on which face of the parent cube) and we say that, for example, if a parent cube is at relative space (x, y, z) and we 
spawn the new cube on the +x surface, then the child cube has relative space (x+1, y, z). 

This significantly speeds up the creation and generation process for cubes, as a problem that typically costs O(n^2) (volume collision checking) is reduced to O(n) (checking if a value is in a list).
The accuracy does not go down significantly except in the event of extreme cubes (pancake cubes or tower cubes) spawning next to each other, which is inherently unlikely.

# MORPHOSPACE
Technically, all bodies are possible. This generates a 3D random branching of limbs, so it can theoretically evolve in an infinite manner of ways. 
Brains are a similar idea. The generation of sensors and motors is at random, which can generate all random neural networks within physical constraints.
Similarly, all sensors can affect all motors, as this amount is random too. 

# DIAGRAM
An important note about the diagram: Since states within solution objects are saved, evolutions are changes to the solution state, and then 
only afterwards is object data sent to pyrosim. Therefore, Evolution in the diagram is covered in "addition of cube" and "deletion of cube" and "mutation modification of weight" sections of the diagram. 
Note, however, "addition of cube" also serves as the k+1 step of creating creatures, however it would be inappropriate to differentiate the two addition functions, since they are the same.
![alt text](https://github.com/AlexChen0/ArtificialLifeAC/blob/main/3DCreatureLogic.jpg)

# FITNESS GRAPH OVER TIME
This fitness graph shows the evolution of 5 seeds of creature over 20 generations. 
![alt text](https://github.com/AlexChen0/ArtificialLifeAC/blob/main/FitnessFunctions.png)
