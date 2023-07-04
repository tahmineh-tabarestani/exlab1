#! /usr/bin/env python
"""
.. module:: controller
    :platform: Unix
    :synopsis: the controller python script in topological_map_robot_control package
    
.. moduleauthor:: Tahmineh Tabarestani <Tahmineh.tbi@gmail.com>

This node sets the client pose by utilizing the "anm.SERVER_SET_POSE" service and employs the execute callback to generate feedback while iterating through each set of points.

Service:
The SET_POSE service is employed to adjust the robot's position, utilizing the 'robot_state' node.
"""




import rospy
import sys
sys.path.append('/root/ros_ws/src/exp_assignment/src')
# Import constant name defined to structure the architecture.
from utilities import architecture_name_mapper as anm
# Import the ActionServer implementation used.
# This is required to pass the `Action_controller` type for instantiating the `SimpleActionServer`.
from actionlib import SimpleActionServer
# Import custom message, actions and services.
from exp_assignment.msg import ControlFeedback, ControlResult
from exp_assignment.srv import SetPose
import tmrc  

# A tag for identifying logs producer.
LOG_TAG = anm.NODE_CONTROLLER

class ControllingAction(object):
    """
    This class has:
        1. An SimpleActionServer to simulate motion controlling.
        2. Given a plan as a set of via points, it simulate the movements to reach each point. 
        3. This server updatesthe current robot position stored in the `robot-state` node.
    """

    def __init__(self):
        # Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer(anm.ACTION_CONTROLLER,
                                      tmrc.msg.ControlAction,
                                      execute_cb=self.execute_callback,
                                      auto_start=False)
        self._as.start()

    def execute_callback(self, goal):
        """
        This function does:
            1. The callback invoked when a client set a goal to the `controller` server.
            2. Requires a list of via points (i.e., the plan), and it simulate a movement through each point. 
            3. The related robot position is updated in the `robot-state` node.

        Arg:
            goal(float): [x,y] coordinates of next destination of robot

        """
        # Construct the feedback and loop for each via point.
        feedback = ControlFeedback()
        rospy.loginfo(anm.tag_log('Server is controlling...', LOG_TAG))
        for point in goal.via_points:
            # Check that the client did not cancel this service.
            if self._as.is_preempt_requested():
                rospy.loginfo(anm.tag_log('Service has been cancelled by the client!', LOG_TAG))
                # Actually cancel this service.
                self._as.set_preempted()
                return
                
            # Publish a feedback to the client to simulate that a via point has been reached. 
            feedback.reached_point = point
            self._as.publish_feedback(feedback)
            # Set the new current position into the `robot-state` node.
            _set_pose_client(point)
            # Log current robot position.
            log_msg = f'Reaching point ({point.x}, {point.y}).'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        # Publish the results to the client.
        result = ControlResult()
        result.reached_point = feedback.reached_point
        rospy.loginfo(anm.tag_log('Motion control successes.', LOG_TAG))
        self._as.set_succeeded(result)
        return  # Succeeded.

def _set_pose_client(pose):
    """
    A simple function to update the current robot `pose` stored in the `robot-state` node.
    This method is performed for each point provided in the action's server feedback.

    Arg:
        pose(float): [x,y] coordinates of current position 
    """
    # Eventually, wait for the server to be initialised.
    rospy.wait_for_service(anm.SERVER_SET_POSE)
    try:
        # Log service call.
        log_msg = f'Set current robot position to the `{anm.SERVER_SET_POSE}` node.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        # Call the service and set the current robot position.
        service = rospy.ServiceProxy(anm.SERVER_SET_POSE, SetPose)
        service(pose)  # The `response` is not used.
    except rospy.ServiceException as e:
        log_msg = f'Server cannot set current robot position: {e}'
        rospy.logerr(anm.tag_log(log_msg, LOG_TAG))


if __name__ == '__main__':
    # Initialise the node, its action server, and wait.   
    rospy.init_node(anm.NODE_CONTROLLER, log_level=rospy.INFO)
    server = ControllingAction()
    rospy.spin()
