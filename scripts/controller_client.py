#!/usr/bin/env python
"""
.. module:: controller_client
    :platform: Unix
    :synopsis: the controller_client python script in topological_map_robot_control package
    
.. moduleauthor:: Tahmineh Tabarestani <Tahmineh.tbi@gmail.com>

Brief explanation of this node:
	This node utilizes the controller_client function to establish communication with the 	SimpleActionServer and cancels the goal if necessary. Additionally, it creates an 		action client to send a goal to the ControlAction. The controller_client_callback 		function is utilized to update the data.

Subscribe to:
 "/path" topic to receive the result from the controller_client_callback function.

"""

import rospy
import time
# Import the ActionServer implementation used.
# This is required to pass the `Action_controller` type for instantiating the `SimpleActionClient`.
from actionlib import SimpleActionClient
# Import the messages used by services and publishers.
from tmrc.msg import Point, PlanGoal
import sys
sys.path.append('/root/ros_ws/src/exp_assignment/src')
# Import constant name defined to structure the architecture.
from utilities import architecture_name_mapper as anm
import tmrc  

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_CONTROLLER_CLIENT

def controller_client_callback(data):
    """
    Function to substitute control result in controller client.

    Arg:
        data(float): coordinates of target room
    """
    control_result =  controller_client(data)
    print(control_result)

def controller_client(goal):
    """
    This function it uses the SimpleActionClient to connect to server in order to send, cancel and waiting for server and
    create an action client called "planner_client" with action definition file "tmrc.msg.ControlAction"
    
    Arg:
        goal(float)
    """
    # Create an action client called "planner_client" with action definition file "tmrc.msg.ControlAction"
    client = SimpleActionClient(anm.ACTION_CONTROLLER,tmrc.msg.ControlAction)
    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()
    print("waiting for controller server")
    # Sends the goal to the action server.
    client.send_goal(goal)
    print("waiting for robot to reach the target wihthin 30 seconds")
    finished_before_timeout = client.wait_for_result(timeout=rospy.Duration(30))
    # detects if the target is reached before timeout
    if finished_before_timeout:
        print("Target Reached")
        time.sleep(3)
        return client.get_result()
    else:
        print("Action did not finish before time out!")
        time.sleep(3)
        client.cancel_all_goals()

if __name__ == '__main__':
    # Initialise this node.
    rospy.init_node(anm.NODE_CONTROLLER_CLIENT, log_level=rospy.INFO)
    #Subscribe to: /path to get result from controller_client_callback function.
    rospy.Subscriber('/path', tmrc.msg.PlanResult, controller_client_callback)
    rospy.spin()
