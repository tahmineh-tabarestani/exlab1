# exlab1
![200863752-0da59da4-dac3-4031-a93c-6fab05ed9516](https://github.com/tahmineh-tabarestani/exlab1/assets/80887743/aad21862-901c-4470-bb65-c9e07e01fc36)



#Introduction:
The concept behind this project involves the development of a robot capable of moving between different locations. However, as it moves, it consumes its battery power and necessitates returning to its initial location for recharging. In order to ensure that the robot visits all locations, it is crucial to record the time it takes to travel from one location to another.

Alternate Tools:

In order to tackle this problem, I employed a variety of tools, including:

Protègè: This tool was utilized to construct a topological map and establish the logical framework for the project.

aRMOR: I utilized aRMOR to load the topological map and implement manipulation and query classes, enabling efficient interaction with the map.

Arch_skeleton: This tool proved instrumental in constructing a plan that facilitates movement between the predefined points on the map.

SMACH viewer: To monitor the status of the robot, I utilized the SMACH viewer, which provides a visual representation of the robot's actions and progress.


# Software architecture:
![uml assignment1a drawio (1)](https://github.com/tahmineh-tabarestani/exlab1/assets/80887743/9575b9d6-320a-4406-b827-b6cba537d587)


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
#rqt graph!

![201729229-b924a3cf-628f-4e8b-bee4-66fa636f2aea](https://github.com/tahmineh-tabarestani/exlab1/assets/80887743/cfad2d4f-9d41-44db-b3d5-300c5bdbd847)


The finite_state_machine node takes charge of managing the robot's movements by identifying and prioritizing the rooms that haven't been visited for a significant period of time. It compiles a list of these rooms and selects the first room as the target point. The node then sends a post message to the /motion/planner node, providing the target point information.

The planner node receives the target point and generates a path that outlines the optimal route for the robot to traverse from its current location to the destination. This path is then published to the /motion/controller node.

Upon receiving the path, the controller node takes over the task of maneuvering the robot towards the designated destination. It employs various control mechanisms and strategies to navigate the robot along the generated path.

As the robot progresses towards the target room, the finite_state_machine node remains active and continuously updates the /smach_viewer. This ensures that the viewer provides real-time information and visual representation of the robot's current status.

To facilitate communication between the nodes, the GET_POSE and SET_POSE services defined in the /robot_state node are utilized. These services enable the exchange of messages and data related to the robot's position and pose between the nodes, ensuring smooth coordination and synchronization throughout the system.
#Temporal diagram![tempa drawio](https://github.com/tahmineh-tabarestani/exlab1/assets/80887743/9fcf3afc-571f-4eb3-8391-e6e8ccc16d51)


The topological_map_robot_control.launch launch file initiates the system by starting six nodes and one armor service. This service directly interacts with the FSM (Finite State Machine) node and the topological_map.owl map. It enables the FSM node to make modifications, perform checks, and load the topological_map.owl map.

Additionally, the launch file sets the size of the environment within the planner node and specifies the initial position of the robot in the environment using the controller_client node. Parameters are utilized to assign specific values to these settings.

The FSM node publishes the \target_point message to the planner_client node. The planner_client node then utilizes this information to generate a path that incorporates various points to navigate the robot towards the desired goal. The resulting path is published as the \path message to the controller_client node.

The nodes, such as the MOTION PLANNER and MOTION CONTROLLER, work in pairs and establish communication with each other through the SimpleAction protocol. In this setup, the first node acts as the server, while the second node functions as the client. The client node establishes a connection with the server using a CALLBACK mechanism, allowing them to exchange data and instructions. The server, on the other hand, utilizes the FEEDBACK mechanism to monitor and control the status of the client during the execution of tasks

#usage

To create your own repository and set up the necessary environment, follow these steps:

1. Create a ROS workspace and source folder:
   ```
   $ mkdir ros_ws
   $ cd ros_ws
   $ mkdir src
   $ cd src
   ```

2. Fork the package from the desired repository:
   ```
   $ git clone https://github.com/mrhosseini75/tmrc.git
   ```

3. Go back to the root of the workspace:
   ```
   $ cd ..
   ```

4. Build the workspace using `catkin_make`:
   ```
   $ catkin_make
   ```

With these steps, you have created your ROS workspace, added the desired package from the repository, and built the workspace using `catkin_make`. Your workspace is now ready for further development and usage.
Now you can launch program:
$ roslaunch tmrc topological_map_robot_control
#Hypothesis and environment
System's Features:

The final environment is designed with a map size of 10x10, containing seven distinct points representing different locations. The planner node utilizes these points to generate a path, denoted as /path, guiding the robot's movement.

The system keeps track of the time the robot passes through each room using the visitedAt method. This information is used to update the list of rooms to be visited. If more than seven seconds have elapsed since the robot's last visit to a room, that room is classified as "urgent" and added to the urgent list.

System's Limitations:

The robot can only recharge in room E and is limited to a maximum charge capacity of 20 percent. Battery consumption occurs when the robot is in motion, but it does not deplete while stationary in a room.

If the robot's battery level drops to 7 percent, the system identifies room E as the first urgent case for the robot to visit.

Possible Technical Improvements:

To enhance visualization of the robot's behavior, it is possible to incorporate a world representation that provides a more comprehensive view of the robot's actions and surroundings.

Considering the inclusion of walls and obstacle avoidance, the world representation and /laser_scan topic can be utilized to implement an obstacle_avoidance node, ensuring that the robot navigates effectively in the presence of obstacles.

Currently, there is no control over the robot's speed. To address this, the /cmd_vel topic can be utilized to generate velocity commands, allowing for better control and adjustment of the robot's movement speed.
