# RUNNING THE PROGRAM
simply navigate to this directory and use the command:

python3 main.py

# VIDEO LINKS
Note: I mistakenly had a > and < swapped for the entirety of the 50k sims. So if you run the robot, most of em actually end up like this:

An excellent robot: https://youtu.be/xtETCiprSBk

That said, here's the trailer: https://youtu.be/XcUJ9kByjus

Full video: https://youtu.be/mbgcQWo6jlU

# A NOTE ON EXTRA "NEAT VIDEOS AND GRAPHS"
This readme has one graph of the final results, and no videos. For the host of videos and graphs, see the
artificialLifeFinalProjectStuff directory, where its subdirectories are marked to indicate graphs and videos.

# METHODS -- CREATION, MUTATION, SELECTION, EVOLUTION
The simulation operates on Parallel Hill Climbing. This is to say that a lot of robots are created, then they are mutated, then the best ones are selected, and then all future robots evolve from there.

The following diagram shows a basic example of robot creation:

![alt text](https://github.com/AlexChen0/ArtificialLifeAC/blob/main/BodyBrainGen.jpg)

This is to say that a robot first begins as a lone block. Afterwards, the computer decides how many segments the robot should have, and for each segment which ones should have sensors, and which joints should have motors. 
For each new block, a face of an existing block is chosen that does not have a block already attached to it. Once this is selected, then a joint is made, and the cube is made afterwards. This is all done completely randomly

Brains are then generated from a mapping of sensors to motors. Each sensor has a corresponding weight to each motor, meaning that individual weights can be allocated and changed. 
Since all the sensors and motors are already determined in body generation, this data is fetched from storage, and used to generate the brain. No further work is necessary.

At each stage of parallel hill climbing, mutations are made from the parents to the children. The following diagram shows the possibilities of mutation:

![alt text](https://github.com/AlexChen0/ArtificialLifeAC/blob/main/BodyBrainMutation.jpg)

Mutations can take one of three forms: adding a block, removing a block, or changing a weight. Adding a block happens identically to body creation, and removing a block simply removes a block and its associated joint. 
Though there is an additional caveat: The only blocks that can be removed are blocks that are not parents of any joints. Finally, changing weights is done by directly changing the array of weights randomly as well.

Finally, Selection and Evolution work in the following way: 

![alt text](https://github.com/AlexChen0/ArtificialLifeAC/blob/main/SelectionEvolution.jpg)

This is where we introduce the concept of fitness. For a mutation to evolve into the robot lineage, it must be "more successful" than other creatures, AND more successful than the parent. 
Therefore, I define fitness to mean the amount a robot can move, within a roughly 10 second time period (translating to 1500 simulation steps), in a direction away from the camera.
This serves to favor creatures that have steady movement over creatures that take one leap and fail briefly afterwards. 

The fitness, then, can be easily tracked by finding the location of the robot at the final step. If the fitness qualifies as above, then the mutation is successful, and incorporated into all future generations. At this point, the creature has evolved. 

# METHODS-- HOW THE CODE WORKS
The call to main.py runs a call to search.py (technically, you can just run search.py, but main.py is the "standard"), 
which then sets up the parallel hill climbing logic that the code uses. Search.py instantiates a parallelHillClimber, 
which is simply instructed to "evolve". When it does so, it first creates parents to begin the evolution, with all parents being a "solution" object, in solution.py.

These parents store a lot of data, which serves as a representation of "genetic code" for the parent that can mutate over time. To keep things brief, 
each parent at their core store their list of sensor blocks, nonsensor blocks, and motor joints, and the instructions for how sensitive each motor is to each sensor. 
The remainder of information stored is logistical, and serves as information to speed up the process of creating bots. 

The code itself sacrifices perfect accuracy for approximations, with rare instances of not-great performance, typically when large blocks and tiny blocks
are next to each other. This allows generations to run very, very fast (with all 50,000 sims being run within 2.5 hours). Robots are approximated, their data stored,
and finally sent to pyrosim to construct the actual bots in simulation. 

This is where simulation.py and simulate.py come in. As the name suggests, they operate the robot and the world that are stored, and with all the pyrosim data already available,
the robot data is passed to robot.py, which then passes logic to motor.py and sensor.py. Once the simulation is complete, robot.py returns the fitness data, which gets propagated back up the chain 
to the parallelhillclimber, so the next evolution step can happen. 

# SOURCES
All information for setting up the bots was found in the reddit ludobots instructions starting here:

https://www.reddit.com/r/ludobots/wiki/installation/

# RESULTS
Parallel hill climbing works. If nothing else, this diagram essentially acts as a TL:DR of the entire results section:

![alt text](https://github.com/AlexChen0/ArtificialLifeAC/blob/main/artificialLifeFinalProjectStuff/AllGraphs/run20.png)

This plot shows evolution of 10 creatures in a random seed, that occur through 250 generations (for all the graphs, see the final project directory, and the AllGraphs subdirectory). 
Similarly, a before/after can be found in the final project directory of this project, labeled "Teaser.mp4"

At a high level, the bodies looked nothing like what they began as, and evolution often happened several times within each robot in a given seed, though never happened 
from one generation to the next. And from this, robots found all different kinds of ways to move away from the camera: some fidgeted away, 
others rotated to get away, some were able to crawl using a pseudo-leg, and finally one was even able to cartwheel. 

However, evolution did get stuck. And it got stuck quite a bit. Bad seeds result in failed robots, ones where despite evolution taking place, did not end up 
too much better than the original. These robots appeared to have the body parts in place to succeed, but likely did not have the brain to do so. 

This makes sense in a lot of ways. Evolution of a new block can only go on a limited number of places, and each area a new block 
can spawn can be vastly different depending on the weights it has placed from the various sensors. Therefore, finding the right mix 
of sensors is nearly impossible.

That being said, successes also happened a lot, and for that reason additional generations did not appear to be needed. 