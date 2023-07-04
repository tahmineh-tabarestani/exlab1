#! /usr/bin/env python
"""
.. module:: planner
    :platform: Unix
    :synopsis: the controller_ python script in topological_map_robot_control package
    
.. moduleauthor:: Tahmineh Tabarestani <Tahmineh.tbi@gmail.com>

Brief explanation of this node:
    Get client pose to anm.SERVER_GET_POSE service and use execute callback to 
    construct the feedback and loop for each set of points.

Service:
    GET_POSE to get position from 'robot_state' node

Param:
        Get PLANNER_POINTS
        Get ENVIRONMENT_SIZE 
"""

import rospy
import sys
sys.path.append('/root/ros_ws/src/exp_assignment/src')
# Import constant name defined to structure the architecture.
from utilities import architecture_name_mapper as anm
# Import the ActionServer implementation used.
# This is required to pass the `PlanAction` type for instantiating the `SimpleActionServer`.
from actionlib import SimpleActionServer
# Import custom message, actions and services.
from exp_assignment.msg import Point, PlanFeedback, PlanResult
from exp_assignment.srv import GetPose
import tmrc  

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_PLANNER

class PlaningAction(object):
    """
    This class has:
        1. An action server to simulate motion planning.
        2. Given a target position, it retrieve the current robot position from the `robot-state` 
        node, and return a plan as a set of via points.

    """

    def __init__(self):
        # Get based parameters used by this server
        self._plan_points = rospy.get_param(anm.PARAM_PLANNER_POINTS, [2, 8])
        self._environment_size = rospy.get_param(anm.PARAM_ENVIRONMENT_SIZE)
        # Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer(anm.ACTION_PLANNER, 
                                      tmrc.msg.PlanAction, 
                                      execute_cb=self.execute_callback, 
                                      auto_start=False)
        self._as.start()
        # Log information.
        log_msg = (f'`{anm.ACTION_PLANNER}` Action Server initialised. It will create path with a number of point '
                   f'spanning in [{self._plan_points[0]}, {self._plan_points[1]}). Each point will be generated ')
      
    def execute_callback(self, goal):
        """
        This function does:
            1. The callback invoked when a client set a goal to the `planner` server.
            2.  Will return a list of points (i.e., the plan) when the fist point is 
                the current robot position (retrieved from the `robot-state` node), while 
                the last point is the `goal` position (given as input parameter).
        Args:
            goal (float): target position of robot
        """
        # Get the input parameters to compute the plan, i.e., the start (or current) and target positions.
        start_point = _get_pose_client()
        target_point = goal.target
        
        # Initialise the `feedback` with the starting point of the plan.
        feedback = PlanFeedback()
        feedback.via_points = []
        feedback.via_points.append(start_point)
        feedback.via_points.append(target_point)
        # Publish the feedback and wait to simulate computation.
        self._as.publish_feedback(feedback)

        log_msg = f'Server is planning ...'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        # Publish the results to the client.        
        result = PlanResult()
        result.via_points = feedback.via_points
        self._as.set_succeeded(result)
        log_msg = 'Motion plan succeeded with plan: '
        log_msg += ''.join('(' + str(point.x) + ', ' + str(point.y) + '), ' for point in result.via_points)
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

def _get_pose_client():
    """
    This function retrieve the current robot pose by the `state/get_pose` server of the `robot-state` node.

    Returns:
        msg: pose of client
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


if __name__ == '__main__':
    # Initialise the node, its action server, and wait.    
    rospy.init_node(anm.NODE_PLANNER, log_level=rospy.INFO)
    server = PlaningAction()
    rospy.spin()
