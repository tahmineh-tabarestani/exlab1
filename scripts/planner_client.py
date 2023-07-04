#!/usr/bin/env python
"""
.. module:: planner_client
    :platform: Unix
    :synopsis: the controller_client python script in topological_map_robot_control package
    
.. moduleauthor:: Tahmineh Tabarestani <Tahmineh.tbi@gmail.com>

Brief explanation of this node:
    Use controller_client function to communicate with SimpleActionServer and
    cancels goal in case of need, additionaly create an action client to send
    plan to PlanAction. Use planner_client_callback to substitute new data.

Service:
    SET_POSE to set just inital position to 'robot_state' node


Publishe :
    /path to PlanResult

Subscribe to:
        /target_point to PlanGoal

"""


import rospy
import time
# Import the ActionServer implementation used.
# This is required to pass the `PlanAction` type for instantiating the `SimpleActionClient`.
from actionlib import SimpleActionClient
# Import the messages used by services and publishers.
from tmrc.msg import Point, PlanGoal
from tmrc.srv import SetPose
import sys
sys.path.append('/root/ros_ws/src/exp_assignment')
# Import constant name defined to structure the architecture.
from utilities import architecture_name_mapper as anm
import tmrc 

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_PLANNER_CLIENT

def planner_client_callback(data):
    """
    Function to substitute plan result in planner client.
    
    Args:
        data (float): [x,y] coordinates of target
    """
    plan_result = planner_client(data.target.x, data.target.y)
    print(plan_result)

    pub.publish(plan_result)

def planner_client(x, y):
    """
    This function it uses the SimpleActionClient to connect to server in order to send, cancel and waiting for server and
    create an action client called "planner_client" with action definition file "tmrc.msg.PlanAction"

    Args:
        x (float): x-coordinate of target
        y (float): y-coordinate of target

    Returns:
        msg: get result
    """
    # Create an action client called "planner_client" with action definition file "tmrc.msg.PlanAction"
    client = SimpleActionClient(anm.ACTION_PLANNER,tmrc.msg.PlanAction)
    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()
    print("waiting for planner server")
    goal = PlanGoal()
    goal.target.x = x
    goal.target.y = y
    # Sends the goal to the action server.
    client.send_goal(goal)
    print("waiting for planner to find the path")
    finished_before_timeout = client.wait_for_result(timeout=rospy.Duration(30))
    # detects if the plan is found before timeout
    if finished_before_timeout:
        print("Plan Found")
        time.sleep(3)
        return client.get_result()
    else:
        print("Action did not finish before time out!")
        time.sleep(3)
        client.cancel_all_goals()

if __name__ == '__main__':
    # Initialise this node.
    rospy.init_node(anm.NODE_PLANNER_CLIENT, log_level=rospy.INFO)
    #Subscribe to: /target_point to PlanGoal
    rospy.Subscriber('/target_point', tmrc.msg.PlanGoal, planner_client_callback)
    #Publishe to: /path to PlanResult
    pub = rospy.Publisher('/path', tmrc.msg.PlanResult, queue_size=10)
    rospy.spin()
