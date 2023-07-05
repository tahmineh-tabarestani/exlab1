# exlab1
#Introduction:
The concept behind this project involves the development of a robot capable of moving between different locations. However, as it moves, it consumes its battery power and necessitates returning to its initial location for recharging. In order to ensure that the robot visits all locations, it is crucial to record the time it takes to travel from one location to another.

Alternate Tools:

In order to tackle this problem, I employed a variety of tools, including:

Protègè: This tool was utilized to construct a topological map and establish the logical framework for the project.

aRMOR: I utilized aRMOR to load the topological map and implement manipulation and query classes, enabling efficient interaction with the map.

Arch_skeleton: This tool proved instrumental in constructing a plan that facilitates movement between the predefined points on the map.

SMACH viewer: To monitor the status of the robot, I utilized the SMACH viewer, which provides a visual representation of the robot's actions and progress.


# Reimagined Explanation:

Robot State Node:

This component encompasses a node that facilitates communication between the client and other nodes by providing GET_POSE and SET_POSE services. It enables the client to retrieve and set the position of the robot, as well as interact with other nodes in the system.

Motion Controller:

The motion controller comprises two essential elements: the controller_client and controller nodes. Working in conjunction, these nodes are responsible for coordinating the movement of the robot between different locations. They receive commands and instructions and execute the necessary actions to navigate the robot to its desired destination.

Motion Planner:

The motion planner encompasses two key components: the planner_client and planner nodes. These nodes work together to generate a map of points that dictate the optimal path for the robot to transition from one room to another. The points are defined within the map, which serves as a reference for planning the robot's movements.

aRMOR:

The aRMOR package consists of six nodes, each serving a specific purpose:

Manipulation in Ontology Map: This node allows manipulation of the ontology map, enabling updates and modifications to the map's content.

Query from Ontology Map: This node facilitates querying and retrieving information from the ontology map.

Exception on Ontology Map: This node handles exceptions and error conditions that may arise during interactions with the ontology map.

Architecture Name Mapper: Each node within the aRMOR package is assigned a unique name tag, allowing for easy identification and mapping of the architecture.

Finite State Machine:

Within this node, various functions and operations are defined. It serves as the core component responsible for managing the robot's state and behavior. Additionally, it provides a graphical user interface (GUI) known as the SMACH viewer, which allows for real-time visualization and monitoring of the robot's status
