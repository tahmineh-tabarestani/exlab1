#!/usr/bin/env python
"""
.. module:: finit_state_machine
    :platform: Unix
    :synopsis: the finite_state_machine python script in topological_map_robot_control package
    
.. moduleauthor:: Tahmineh Tabarestani <Tahmineh.tbi@gmail.com>

Brief explanation of this node:
	1. Individuals (ROOM, DOOR, ROBOT) are assigned to their respective classes.
	2. Disjoint individuals (ROOM, DOOR) are identified.
	3. DOORs are assigned to their corresponding ROOMs.
	4. The "last visited times" method is utilized to update the time of visit.
	5. The robot is added to the map and its initial pose is set.
	6. The "urgency threshold" method is employed to ensure all rooms are visited.
	7. The robot's battery level is tracked,and the "charge and discharge" method is used.
	8. The pose client is used to obtain the current location of the robot.
	9. The ontological map is updated.
	10. The robot is moved between different ROOMs.
	11. The smach viewer is executed to display the current state of the robot.

Publishe :
    /target_point to 'planner_client' node 

Service :
    GET_POSE to get robot pose from 'robot_state' node
"""

import rospy
import smach
import smach_ros
import time
from os.path import dirname, realpath
from std_msgs.msg import String
# Import custom message, actions and services.
from tmrc.msg import PlanGoal
from tmrc.srv import GetPose
import tmrc  
import sys
# Import constant name defined to structure the architecture.
sys.path.append('/root/ros_ws/src/exp_assignment')
from utilities import architecture_name_mapper as anm
from utilities.armor_client import ArmorClient



# A tag for identifying logs producer.
LOG_TAG = anm.NODE_FINITE_STATE_MACHINE

# Define global variabiles
robot = ""
client = None
is_in = " "
prev_time = " "
target_room = " "

def add_individuals_to_classes():
    """Function to add all individuals like ROOMs, DOORs, and ROBOT into the class of topological map"""
    # Add indiviuals to class --> ROOM
    client.manipulation.add_ind_to_class("E", "ROOM")
    client.manipulation.add_ind_to_class("C1", "ROOM")
    client.manipulation.add_ind_to_class("C2", "ROOM")
    client.manipulation.add_ind_to_class("R1", "ROOM")
    client.manipulation.add_ind_to_class("R2", "ROOM")
    client.manipulation.add_ind_to_class("R3", "ROOM")
    client.manipulation.add_ind_to_class("R4", "ROOM")

    #Add indiviuals to class --> DOOR
    client.manipulation.add_ind_to_class("D1", "DOOR")
    client.manipulation.add_ind_to_class("D2", "DOOR")
    client.manipulation.add_ind_to_class("D3", "DOOR")
    client.manipulation.add_ind_to_class("D4", "DOOR")
    client.manipulation.add_ind_to_class("D5", "DOOR")
    client.manipulation.add_ind_to_class("D6", "DOOR")
    client.manipulation.add_ind_to_class("D7", "DOOR")

    #Add indiviuals to class --> ROBOT
    client.manipulation.add_ind_to_class("Robot", "ROBOT")      

def disjoint_individuals():
    """Function to disjoint individuals like ROOMs and DOORs in topological map"""
    client.manipulation.disj_inds_of_class("ROOM")
    client.manipulation.disj_inds_of_class("DOOR")

def assign_doors_to_rooms():
    """Function to assign DOORs to ROOMs in topological map"""
    client.manipulation.add_objectprop_to_ind("hasDoor", "R1", "D1")
    client.manipulation.add_objectprop_to_ind("hasDoor", "R2", "D2")
    client.manipulation.add_objectprop_to_ind("hasDoor", "R3", "D3")
    client.manipulation.add_objectprop_to_ind("hasDoor", "R4", "D4")
    client.manipulation.add_objectprop_to_ind("hasDoor", "R4", "D4")
    client.manipulation.add_objectprop_to_ind("hasDoor", "E", "D6")
    client.manipulation.add_objectprop_to_ind("hasDoor", "E", "D7")
    client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D1")
    client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D2")
    client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D5")
    client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D7")
    client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D3")
    client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D4")
    client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D5")
    client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D6")

def add_last_visit_times():
    """Function to update the last time the robot was there"""

    # Time
    now = rospy.get_rostime()
    client.manipulation.add_dataprop_to_ind("visitedAt", "R1", "Int", str(now.secs - 70))
    client.manipulation.add_dataprop_to_ind("visitedAt", "R2", "Int", str(now.secs - 50))
    client.manipulation.add_dataprop_to_ind("visitedAt", "R3", "Int", str(now.secs - 30))
    client.manipulation.add_dataprop_to_ind("visitedAt", "R4", "Int", str(now.secs - 20))
    client.manipulation.add_dataprop_to_ind("visitedAt", "E", "Int", str(now.secs))
    client.manipulation.add_dataprop_to_ind("visitedAt", "C1", "Int", str(now.secs - 60))
    client.manipulation.add_dataprop_to_ind("visitedAt", "C2", "Int", str(now.secs - 40))  

def add_robot():
    """Function to add robot in topological map"""
    client.manipulation.add_objectprop_to_ind("isIn", "Robot", "E")
    # Time
    now = rospy.get_rostime()
    client.manipulation.add_dataprop_to_ind("now", "Robot", "Long", str(now.secs))

def add_urgency_threshold():
    """Function to creat an urgency threshold in case more than 
        7 secs have passed since the last time the robot was there """
    client.manipulation.add_dataprop_to_ind("urgencyThreshold", "Robot", "Int", "7")

def add_battery_level():
    """Function to create battery level for robot, battery level of robot is 20 percent."""
    client.manipulation.add_dataprop_to_ind("batteryLvl", "Robot", "Int", "20")

def charge_battery():
    """Function to charge battery of robot. If battery level of robot is equal to 20 function stops."""
    prev_battery_lvl = cut_dataprop_list(client.query.dataprop_b2_ind("batteryLvl", "Robot"))[0]
    if  int(prev_battery_lvl) + 10 < 20:
        battery_lvl = int(prev_battery_lvl) + 5
    else:
        battery_lvl = 20
    client.manipulation.replace_dataprop_b2_ind("batteryLvl", "Robot", "Int", str(battery_lvl), prev_battery_lvl)

def consume_battery():
    """Function to consume battery of robot when robot move between ROOMs."""
    prev_battery_lvl = cut_dataprop_list(client.query.dataprop_b2_ind("batteryLvl", "Robot"))[0]
    battery_lvl = int(prev_battery_lvl) - 1
    client.manipulation.replace_dataprop_b2_ind("batteryLvl", "Robot", "Int", str(battery_lvl), prev_battery_lvl)

def get_pose_client():
    """
    Function to retrieve the current robot pose by the `state/get_pose`
     server of the `robot-state` node.
    """
    # Eventually, wait for the server to be initialised.
    rospy.wait_for_service(anm.SERVER_GET_POSE)
    try:
        # Call the service and get a response with the current robot position.
        service = rospy.ServiceProxy(anm.SERVER_GET_POSE, GetPose)
        response = service()
        pose = response.pose
        # Log service response.
        log_msg = f'Retrieving current robot position from the `{anm.NODE_ROBOT_STATE}` node as: ({pose.x}, {pose.y}).'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        return pose
    except rospy.ServiceException as e:
        log_msg = f'Server cannot get current robot position: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))

def get_location():
    """
    Function to detect robot current location and replace new location with previous location.
    
    Return:
        is_in(string): current location of robot
    """
    global is_in
    now = rospy.get_rostime()
    position = get_pose_client()

    if position.y <= 2:
        is_in = "E"
        prev_time = cut_dataprop_list(client.query.dataprop_b2_ind("visitedAt", "E"))[0]
        client.manipulation.replace_dataprop_b2_ind("visitedAt", "E", "Int", str(now.secs), prev_time)
        
    elif position.x <= 2 and position.y > 2 and position.y <= 6:
        is_in = "R1"
        prev_time = cut_dataprop_list(client.query.dataprop_b2_ind("visitedAt", "R1"))[0]
        client.manipulation.replace_dataprop_b2_ind("visitedAt", "R1", "Int", str(now.secs), prev_time)
        
    elif position.x <= 2 and position.y > 6:
        is_in = "R2"
        prev_time = cut_dataprop_list(client.query.dataprop_b2_ind("visitedAt", "R2"))[0]
        client.manipulation.replace_dataprop_b2_ind("visitedAt", "R2", "Int", str(now.secs), prev_time)

    elif position.x > 2 and position.x <= 5 and position.y > 2:
        is_in = "C1"
        prev_time = cut_dataprop_list(client.query.dataprop_b2_ind("visitedAt", "C1"))[0]
        client.manipulation.replace_dataprop_b2_ind("visitedAt", "C1", "Int", str(now.secs), prev_time)

    elif position.x > 5 and position.x <= 8 and position.y > 2:
        is_in = "C2"
        prev_time = cut_dataprop_list(client.query.dataprop_b2_ind("visitedAt", "C2"))[0]
        client.manipulation.replace_dataprop_b2_ind("visitedAt", "C2", "Int", str(now.secs), prev_time)

    elif position.x > 8 and position.y > 2 and position.y <= 6:
        is_in = "R3"
        prev_time = cut_dataprop_list(client.query.dataprop_b2_ind("visitedAt", "R3"))[0]
        client.manipulation.replace_dataprop_b2_ind("visitedAt", "R3", "Int", str(now.secs), prev_time)

    elif position.x > 8 and position.y > 6:
        is_in = "R4"
        prev_time = cut_dataprop_list(client.query.dataprop_b2_ind("visitedAt", "R4"))[0]
        client.manipulation.replace_dataprop_b2_ind("visitedAt", "R4", "Int", str(now.secs), prev_time)

    return is_in

def update_ontology():
    """
    Function to update topological map and sync the reasoner. If pass more 
    than 7 secs since the last time robot was there, the location is added 
    to the list of URGENT locations to visit. If battery level of robot less 
    than 7, the robot first has to go to ROOM E to charge.

    Return: 
        target_room(string): next destination to vist  
    
    """


    client.utils.sync_buffered_reasoner()
    loc = get_location()
    print("Battery level: ", cut_dataprop_list(client.query.dataprop_b2_ind("batteryLvl", "Robot"))[0])
    print("Current location: ", loc)
    print("Urgent locations: ", client.query.ind_b2_class("URGENT"))

    now = rospy.get_rostime()
    urgency_threshold = cut_dataprop_list(client.query.dataprop_b2_ind("urgencyThreshold", "Robot"))[0]

    visitedAt_E = cut_dataprop_list(client.query.dataprop_b2_ind("visitedAt", "E"))[0]
    visitedAt_R1 = cut_dataprop_list(client.query.dataprop_b2_ind("visitedAt", "R1"))[0]
    visitedAt_R2 = cut_dataprop_list(client.query.dataprop_b2_ind("visitedAt", "R2"))[0]
    visitedAt_R3 = cut_dataprop_list(client.query.dataprop_b2_ind("visitedAt", "R3"))[0]
    visitedAt_R4 = cut_dataprop_list(client.query.dataprop_b2_ind("visitedAt", "R4"))[0]
    visitedAt_C1 = cut_dataprop_list(client.query.dataprop_b2_ind("visitedAt", "C1"))[0]
    visitedAt_C2 = cut_dataprop_list(client.query.dataprop_b2_ind("visitedAt", "C2"))[0]


    visitedAt_dict = {visitedAt_R1: "R1", visitedAt_R2: "R2", visitedAt_R3: "R3", visitedAt_R4: "R4", visitedAt_C1: "C1", visitedAt_C2: "C2", visitedAt_E: "E"}
    visitedAt_dict = dict(sorted(visitedAt_dict.items()))
    room_list = list(visitedAt_dict.values())
    target_room = room_list[0]
    print("VisitedAt times: ", visitedAt_dict)
    print("Target room: ", target_room)

    if now.secs - int(visitedAt_E) > int(urgency_threshold):
        client.manipulation.add_ind_to_class("E", "URGENT")
    else:
        client.manipulation.remove_ind_from_class("E", "URGENT")

    if now.secs - int(visitedAt_R1) > int(urgency_threshold):
        client.manipulation.add_ind_to_class("R1", "URGENT")
    else:
        client.manipulation.remove_ind_from_class("R1", "URGENT")
    
    if now.secs - int(visitedAt_R2) > int(urgency_threshold):
        client.manipulation.add_ind_to_class("R2", "URGENT")
    else:
        client.manipulation.remove_ind_from_class("R2", "URGENT")
    
    if now.secs - int(visitedAt_R3) > int(urgency_threshold):
        client.manipulation.add_ind_to_class("R3", "URGENT")
    else:
        client.manipulation.remove_ind_from_class("R3", "URGENT")
    
    if now.secs - int(visitedAt_R4) > int(urgency_threshold):
        client.manipulation.add_ind_to_class("R4", "URGENT")
    else:
        client.manipulation.remove_ind_from_class("R4", "URGENT")
    
    if now.secs - int(visitedAt_C1) > int(urgency_threshold):
        client.manipulation.add_ind_to_class("C1", "URGENT")
    else:
        client.manipulation.remove_ind_from_class("C1", "URGENT")
    
    if now.secs - int(visitedAt_C2) > int(urgency_threshold):
        client.manipulation.add_ind_to_class("C2", "URGENT")
    else:
        client.manipulation.remove_ind_from_class("C2", "URGENT")
    
    battery_lvl = cut_dataprop_list(client.query.dataprop_b2_ind("batteryLvl", "Robot"))[0]

    if int(battery_lvl) > int(urgency_threshold):
        return target_room
    else:
        return "E"

def move_robot_to_roomE():
    """Function to move robot to ROOM E"""
    move_robot(3.5, 1.0)

def move_robot_to_room1():
    """Function to move robot to ROOM 1"""
    move_robot(1.0, 4.5)

def move_robot_to_room2():
    """Function to move robot to ROOM 2"""
    move_robot(1.0, 7.5)

def move_robot_to_room3():
    """Function to move robot to ROOM 3"""
    move_robot(9.0, 4.5)

def move_robot_to_room4():
    """Function to move robot to ROOM 4"""
    move_robot(9.0, 7.5)

def move_robot_to_corridor1():
    """Function to move robot to CORRIDOR 1"""
    move_robot(3.0, 4.5)

def move_robot_to_corridor2():
    """Function to move robot to CORRIDOR 2"""
    move_robot(6.5, 4.5)

def move_to_room(room):
    """
    Function to move to target room
    
    Arg:
        room(string): target room
    """
    rospy.loginfo("[RDST] Received request for robot to move to " + room)
    if room   == 'E'     : move_robot_to_roomE()
    elif room == 'R1' : move_robot_to_room1()
    elif room == 'R2'   : move_robot_to_room2()
    elif room == 'R3'   : move_robot_to_room3()
    elif room == 'R4'    : move_robot_to_room4()
    elif room == 'C1'    : move_robot_to_corridor1()
    elif room == 'C2'    : move_robot_to_corridor2()

def move_robot(x, y):
    """
    Function to move the robot to specific coordinate
    
    Args:
        x(float): x-coordinate of the target room
        y(float): y-coordinate of the target room
    """
    goal = PlanGoal()
    goal.target.x = x
    goal.target.y = y
    pub.publish(goal)
    rospy.loginfo("[RDST] Request sento to planner server")

def cut_dataprop(data_prop):
    """ 
    Very simple function that cut the data prop from a string received  from armor.
    
    Return:
        data_prop(float)
    """
    start = 0
    end = data_prop.rfind('^') - 2
    data_prop = data_prop[(start+1) : end]
    return data_prop

def cut_dataprop_list(data_prop_list):
    """
    Very simple function that cut the data prop from a list of strings received  from armor.
    
    Return:
        data_prop(float)
    """
    for i in range(len(data_prop_list)):
        data_prop_list[i] = cut_dataprop(data_prop_list[i])
    return data_prop_list

class RoomE(smach.State):
    """Class to define state Room E """
    def __init__(self):
        """Initialisation function, it should not wait"""
        smach.State.__init__(self, outcomes=['D6','D7', 'stay'],
                             )
        
    def execute(self, userdata):
        """Function called when exiting from the node, it can be blacking"""
        time.sleep(5)
        target_room = update_ontology()
        if target_room == "C1" or target_room == "R1" or target_room == "R2":
            move_to_room('C1')
            consume_battery()
            return 'D7'
        elif target_room == "C2" or target_room == "R3" or target_room == "R4":
            move_to_room('C2')
            consume_battery()
            return 'D6'
        else:
            charge_battery()
            return 'stay'

class Room1(smach.State):
    """Class to define state Room 1 """
    def __init__(self):
        """initialisation function, it should not wait"""
        smach.State.__init__(self, outcomes=['D1', 'stay'],
                             )
        
    def execute(self, userdata):
        """Function called when exiting from the node, it can be blacking"""
        time.sleep(5)
        target_room = update_ontology()
        if target_room != "R1":
            move_to_room('C1')
            consume_battery()
            return 'D1'
        else:
            return 'stay'


class Room2(smach.State):
    """Class to define state Room 2 """
    def __init__(self):
        """Initialisation function, it should not wait"""
        smach.State.__init__(self, 
                             outcomes=['D2', 'stay'],
                             )
        
    def execute(self, userdata):
        """Function called when exiting from the node, it can be blacking"""
        time.sleep(5)
        target_room = update_ontology()
        if target_room != "R2":
            move_to_room('C1')
            consume_battery()
            return 'D2'
        else:
            return 'stay'

class Room3(smach.State):
    """Class to define state Room 3 """
    def __init__(self):
        """Initialisation function, it should not wait"""
        smach.State.__init__(self, 
                             outcomes=['D3', 'stay'],
                            )
        
    def execute(self, userdata):
        """Function called when exiting from the node, it can be blacking"""
        time.sleep(5)
        target_room = update_ontology()
        if target_room != "R3":
            move_to_room('C2')
            consume_battery()
            return 'D3'
        else:
            return 'stay'

class Room4(smach.State):
    """Class to define state Room 4 """
    def __init__(self):
        """Initialisation function, it should not wait"""
        smach.State.__init__(self, 
                             outcomes=['D4', 'stay'],
                            )
        
    def execute(self, userdata):
        """Function called when exiting from the node, it can be blacking"""
        time.sleep(5)
        target_room = update_ontology()
        if target_room != "R4":
            move_to_room('C2')
            consume_battery()
            return 'D4'
        else:
            return 'stay'
    

class Corridor1(smach.State):
    """Class to define state Corridor 1 """
    def __init__(self):
        """Initialisation function, it should not wait"""
        smach.State.__init__(self, 
                             outcomes=['D1','D2','D5','D7','stay'],
                            )
        self.sensor_input = 0

    def execute(self, userdata):
        """Function called when exiting from the node, it can be blacking"""
        time.sleep(5)
        target_room = update_ontology()
        if target_room == "R1":
            move_to_room('R1')
            consume_battery()
            return 'D1'
        elif target_room == "R2":
            move_to_room('R2')
            consume_battery()
            return 'D2'
        elif target_room == "C2" or target_room == "R3" or target_room == "R4":
            move_to_room('C2')
            consume_battery()
            return 'D5'
        elif target_room == "E":
            move_to_room('E')
            consume_battery()
            return 'D7'
        else:
            return 'stay'

class Corridor2(smach.State):
    """Class to define state Corridor 2 """
    def __init__(self):
        """Initialisation function, it should not wait"""
        smach.State.__init__(self, 
                             outcomes=['D3','D4','D5','D6','stay'],
                            )

    def execute(self, userdata):
        """Function called when exiting from the node, it can be blacking"""
        time.sleep(5)
        target_room = update_ontology()
        if target_room == "R3":
            move_to_room('R3')
            consume_battery()
            return 'D3'
        elif target_room == "R4":
            move_to_room('R4')
            consume_battery()
            return 'D4'
        elif target_room == "C1" or target_room == "R1" or target_room == "R2":
            move_to_room('C1')
            consume_battery()
            return 'D5'
        elif target_room == "E":
            move_to_room('E')
            consume_battery()
            return 'D6'
        else:
            return 'stay'
def SMACH_state_machine():

    """Function to create a SMACH state machine and start the introspection server for visualization and execute the state machine."""
    sm = smach.StateMachine(outcomes=[])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('E', RoomE(), transitions={'D7':'C1', 'D6':'C2', 'stay':'E'})
        smach.StateMachine.add('C1', Corridor1(), transitions={'D7':'E', 'D1':'R1', 'D2':'R2', 'D5':'C2', 'stay':'C1'})
        smach.StateMachine.add('C2', Corridor2(), transitions={'D6':'E', 'D3':'R3', 'D4':'R4', 'D5':'C1', 'stay':'C2'})
        smach.StateMachine.add('R1', Room1(), transitions={'D1':'C1', 'stay':'R1'})
        smach.StateMachine.add('R2', Room2(), transitions={'D2':'C1', 'stay':'R2'})
        smach.StateMachine.add('R3', Room3(), transitions={'D3':'C2', 'stay':'R3'})
        smach.StateMachine.add('R4', Room4(), transitions={'D4':'C2', 'stay':'R4'})

    #Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, 'START')
    sis.start()

    #Execute the state machine
    outcome = sm.execute()

    #Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':

    #Initialise this node
    rospy.init_node(anm.NODE_FINITE_STATE_MACHINE, log_level=rospy.INFO)
    
    #Publishes to: /target_point to 'planner_client' node 
    pub = rospy.Publisher('/target_point', tmrc.msg.PlanGoal, queue_size=10)

    #Build topological map
    path = dirname(dirname(realpath(__file__)))
    path = path + "/ontology/"
    client = ArmorClient(robot, "ontoRef")
    client.utils.load_ref_from_file(path + 'robot-ontology.owl', '',
                                    True, 'PELLET',False,False) 
    client.utils.set_log_to_terminal(True)

    # Call functions defined above
    add_individuals_to_classes()
    disjoint_individuals()
    assign_doors_to_rooms()
    add_last_visit_times()
    add_robot()
    add_urgency_threshold()
    add_battery_level()
    # Sync the reasoner
    client.utils.sync_buffered_reasoner()
    SMACH_state_machine()
