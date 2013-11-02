#! /usr/bin/env python

import roslib
roslib.load_manifest('youbot_coll_test')
roslib.load_manifest('json_prolog')

import rospy
import actionlib
import argparse
import json_prolog

from move_base_msgs.msg import *
from actionlib_msgs.msg import GoalID
from youbot_coll_test.srv import Goal, GoalResponse
from std_srvs.srv import Empty, EmptyResponse
from youbot_coll_test.msg import Status


class MoveBaseCoordAction(object):

  """
  Wrapper for the move_base actionlib to be used on KnowRob-enabled robots.

  move_coordinator and json_prolog have to be running on the robot in the same
  namespace as this node. The simple-move package has to be loaded in the 
  knowledge base and the logger node has to be running on the robot.

  A new goal can be set calling the move_coordinator/goal service. On succesfull
  completion of the task, a message is published on the move_coordinator/result.

  The move_coordinator/status_update should be called by any module updating
  the knowledge base. Not doing so will result in the robot getting stuck in some
  unpleasant state.

  """

  _executing = None
  _goal = None

  _client = None
  _status_pub = None
  _prolog = None
  _rob_self = None

  _result = None

  def __init__(self, rob_self):
    # this could be done extracting a property from the knowledge base
    self._rob_self = rob_self

    # start move_base client and wait for the server
    self._client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
    self.loginfo("Waiting for move_base server")
    self._client.wait_for_server()

    # get goals through this service
    rospy.Service('move_coordinator/goal', Goal, self.setup_goal)

    # get move_base updates
    rospy.Subscriber("move_base/result", MoveBaseActionResult, self.new_result)
    # get robot knowledge updates
    rospy.Service('move_coordinator/status_update', Empty, self.check_and_run)

    # publish successfull results
    self._result = rospy.Publisher('move_coordinator/result', MoveBaseActionResult)
    self._status_pub = rospy.Publisher('/' + self._rob_self + '/status', Status)

    # interface to prolog
    self.loginfo("Waiting for json_prolog server")
    rospy.wait_for_service('json_prolog/simple_query')
    self._prolog = json_prolog.Prolog('json_prolog')

    self.loginfo("move_coordinator up and running")



  def setup_goal(self, data):
    """  
    Routine associated with the goal service. Always returns OK.

    Sets up the goal for the move_base actionlib and sends it if
    precedence is respected. It uses the check_and_run subroutine for
    priority checking.

    Preemption is not supported; goals sent during execution generate an error.

    """
    if self._goal is not None:
      raise StandardError('Goal ha been set while another goal is pending')

    goal = move_base_msgs.msg.MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = data.x;
    goal.target_pose.pose.position.y = data.y;
    goal.target_pose.pose.orientation.w = 1.0;

    self.loginfo("New goal - x:" + str(data.x) + ", y:" + str(data.y))

    # set up local goal
    self._goal = goal
    self._status_pub.publish("Moving")
    self.check_and_run(None)

    return GoalResponse("OK")

  def check_and_run(self, data):
    """
    Routine associated with the status_update service. Only returns an EmptyResponse
    if called as a ROS service.

    The method first stops the robot, then checks the robot's knowledge base 
    for other moving robots in the environment and either sends on its goal or
    stops witing for new updates.

    """
    if self._goal is not None:  # otherwise, no bother
      #stop current movement
      pub = rospy.Publisher('move_base/cancel', GoalID)
      pub.publish() # an empty cancel erase all goals
      self.loginfo("Robot stopped. Calculating priorities")

      # get moving robots and their ids
      q  = "rdf(R, move:'isInMovingStatus', move:'Moving'), "
      q += "rdf(R, move:'hasRobotId', ID)"
      solutions = self._prolog.query(q).solutions()
      
      priority = True # hoping

      try:
          while True:
              rob = solutions.next()

              # youbot0 has priority over youbot[>0]
              # condition can be obviously changed as needed
              if int(self._rob_self[-1:]) > int(rob['ID'][-1:]):
                priority = False # damn
      except StopIteration:   
          pass

      if priority:
        self._executing = True
        self._goal.target_pose.header.stamp = rospy.Time.now()

        self.loginfo("priority OK. Going on with the goal")
        self._client.send_goal(self._goal)

      else:
        self._executing = False
        self.loginfo("NO priority. Waiting")


    if data is not None:
      return EmptyResponse()

  def new_result(self, data):
    """
    Routine associated with the result topics.

    Forwards the result message and reset the module if necessary.
    """
    # TODO: check more accurately
    if data.status.status == 3: # means success
      
      self._goal = None
      self._executing = False
      self._status_pub.publish("Still")

      self.loginfo("Done. Ready for another task")

      self._result.publish(data)

  def loginfo(self, s):
    rospy.loginfo("[" + self._rob_self + "] " + s)

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument("--rid", help="id for the running robot")
  args = parser.parse_args(rospy.myargv()[1:])


  rospy.init_node('move_coordinator')
  MoveBaseCoordAction(args.rid)
  rospy.spin()
