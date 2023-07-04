#!/usr/bin/env python
"""
.. module:: robot_state
    :platform: Unix
    :synopsis: the controller_client python script in topological_map_robot_control package
    
.. moduleauthor:: Tahmineh Tabarestani <Tahmineh.tbi@gmail.com>

Brief explanation of this node:
    This node has main function to set_pose and get_pose. Use service to generate data and
    communicate with other node and also set the initial pose of robot use parameters.

Service:
        GET_POSE
        SET_POSE
        
Param:
    Get param from '/state/initial_pose'

"""

import rospy
import sys
sys.path.append('/root/ros_ws/src/exp_assignment/src')
# Import constant name defined to structure the architecture.
from utilities import architecture_name_mapper as anm
# Import the messages used by services and publishers.
from std_msgs.msg import Bool
from exp_assignment.msg import Point
from exp_assignment.srv import GetPose, GetPoseResponse, SetPose, SetPoseResponse

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_ROBOT_STATE


class RobotState:
    """
    The node manager class:
        This class defines two services to get and set the current robot pose and next robot pose.
    """


    def __init__(self):
        # Initialise this node.
        # Initialise robot position.
        rospy.init_node(anm.NODE_ROBOT_STATE, log_level=rospy.INFO)
        self._pose = Point()
        # Set initial pose of robot in ROOM E
        start_point_param = rospy.get_param("/state/initial_pose")
        self._pose.x = start_point_param[0]
        self._pose.y = start_point_param[1]
        
        # Define services.
        rospy.Service(anm.SERVER_GET_POSE, GetPose, self.get_pose)
        rospy.Service(anm.SERVER_SET_POSE, SetPose, self.set_pose)

    def set_pose(self, request):
        """
        This function is the `robot/set_pose` service implementation.
        The `request` input parameter is the current robot pose to be set,
        as given by the client. This server returns an empty `response`.

        Args:
            request (float): [x,y] coordinates of client pose

        Returns:
            float: set postion response
        """
        if request.pose is not None:
            # Store the new current robot position.
            self._pose = request.pose
            # Log information.
            self._print_info(f'Set current robot position through `{anm.SERVER_SET_POSE}` '
                             f'as ({self._pose.x}, {self._pose.y}).')
        else:
            rospy.logerr(anm.tag_log('Cannot set an unspecified robot position', LOG_TAG))
        # Return an empty response.
        return SetPoseResponse()

    def get_pose(self, request):
        """
        This function is the `robot/get_pose` service implementation.
        The `request` input parameter is given by the client as empty. Thus, it is not used.
        The `response` returned to the client contains the current robot pose.

        Args:
            request (float): [x,y] coordinates of client pose

        Returns:
            float: get position response
        """
        # Log information.
        if self._pose is None:
            rospy.logerr(anm.tag_log('Cannot get an unspecified robot position', LOG_TAG))
        else:
            log_msg = f'Get current robot position through `{anm.SERVER_GET_POSE}` as ({self._pose.x}, {self._pose.y})'
            self._print_info(log_msg)
        # Create the response with the robot pose and return it.
        response = GetPoseResponse()
        response.pose = self._pose
        return response
    # This is done to allow an intuitive usage of the keyboard-based interface.
    def _print_info(self, msg):
        rospy.loginfo(anm.tag_log(msg, LOG_TAG))

if __name__ == "__main__":
    # Instantiate the node manager class and wait.
    RobotState()
    rospy.spin()
