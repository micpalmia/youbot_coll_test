#!/usr/bin/env python

import roslib
roslib.load_manifest('json_prolog')
roslib.load_manifest('youbot_coll_test')

import rospy
import json_prolog
import time
import argparse

from functools import partial

from nav_msgs.msg import Odometry
from std_msgs.msg import String
from youbot_coll_test.msg import Status
from std_srvs.srv import Empty

class RobotLogger(object):

    """
    A proof of concept interface between the outer world and the robot's knowledge.

    json_prolog has to be running on the robot in the same namespace as this node in
    order for this module to work. The simple-move package has to be loaded in the 
    knowledge base.

    For each of the robots provided in the constructor or passed from command line,
    the logger subscribes to the odom and status topics. While the odom topic provides
    the usual Odometry messages, the Status topic provides a special message of type Status
    that can only be 'Moving' or 'Still'.

    The logger logs the position of each robot, apart for itself, every LOG_TIMEOUT
    milliseconds in the KnowRob knowledge base. It also logs the status of each robot
    whenever it gets updated. In this occasion it also calls the move_coordinator/status_update
    service. This shall be made more general, with any robot module being able to know when such
    an update to the knowledge base is made.

    """

    LOG_TIMEOUT = 10000

    _rob_self = None
    _rob_list = {}

    _rob_instance = {}
    _rob_timing = {}

    _prolog = None

    def __init__(self, rob_self, rob_list):

        self._rob_self = rob_self
        self._rob_list = rob_list
        
        self.loginfo("Waiting for json_prolog server")
        rospy.wait_for_service('json_prolog/simple_query')
        self._prolog = json_prolog.Prolog("json_prolog")

        # for each robot in the robot list
        for robot in [x for x in self._rob_list if x != self._rob_self]:
            self._rob_timing[robot] = 0.00
            rospy.Subscriber("/" + robot + "/odom" , Odometry, self.position_callback(robot))
            rospy.Subscriber("/" + robot + "/status" , Status, self.status_callback(robot))

        self.loginfo("robot_logger up and running")

    def position_callback(self, s):
        return partial(self.position_forwarder, sender=s)

    def position_forwarder(self, data, sender):
        """
        Stores the received position in the knowledge base. If the sender is not known
        a new instance is created for it.

        """
        if self.current_milli_time() - self._rob_timing[sender] > self.LOG_TIMEOUT:
            q = self.quaternionToMatrix(data.pose.pose);
            if not sender in self._rob_instance:
                qr = "rdf_instance_from_class(move:'YouBot', O), "\
                     "rdf_assert(O, move:'hasRobotId', " + sender + "), "\
                     "rdf_assert(O, move:'isInMovingStatus', move:'Still')"
                query = self._prolog.query(qr)
                inst = query.solutions().next()['O']
                query.finish()

                self._rob_instance[sender] = inst
                self.loginfo("Logged instance of " + sender)
            
            qr = "create_perception_instance(['ProgrammedPerception'], Perception), "\
                  "set_object_perception('" + self._rob_instance[sender] + "', Perception), "\
                  "set_perception_pose(Perception, " + str(q) + ")"

            query = self._prolog.query(qr)
            query.finish()
            
            self._rob_timing[sender] = self.current_milli_time()

    def status_callback(self, s):
        return partial(self.status_forwarder, sender=s)

    def status_forwarder(self, data, sender):
        """
        Stores the status of the sender upon receiving it. move_coordinator is notified
        with an Empty message.
        
        """

        sdr = self._rob_instance[sender]
        qr = "rdf_retractall('" + sdr + "', move:'isInMovingStatus', X), "\
             "rdf_assert('" + sdr + "', move:'isInMovingStatus', move:'" + data.status + "')"
        
        query = self._prolog.query(qr)
        query.solutions()
        query.finish()

        try:
            status_update = rospy.ServiceProxy('move_coordinator/status_update', Empty)
            status_update()
        except rospy.ServiceException:
            self.loginfo("move_coordinator service not available for status updates")


    def current_milli_time (self):
        return int(round(time.time() * 1000))

    def quaternionToMatrix(self, pose):
        q = pose.orientation
        p = pose.position
        m = [0] * 16

        xx = q.x * q.x;
        xy = q.x * q.y;
        xz = q.x * q.z;
        xw = q.x * q.w;

        yy = q.y * q.y;
        yz = q.y * q.z;
        yw = q.y * q.w;

        zz = q.z * q.z;
        zw = q.z * q.w;

        m[0]  = 1 - 2 * ( yy + zz );
        m[1]  =     2 * ( xy - zw );
        m[2]  =     2 * ( xz + yw );
        m[3]  = p.x;

        m[4]  =     2 * ( xy + zw );
        m[5]  = 1 - 2 * ( xx + zz );
        m[6]  =     2 * ( yz - xw );
        m[7]  = p.y;

        m[8]  =     2 * ( xz - yw );
        m[9]  =     2 * ( yz + xw );
        m[10] = 1 - 2 * ( xx + yy );
        m[11]=p.z;

        m[12]=0;
        m[13]=0;
        m[14]=0;
        m[15]=1;
        return m;

    def loginfo(self, s):
        rospy.loginfo("[" + self._rob_self + "] " + s)

if __name__ == '__main__':
    # get the namespace for the robot to be operated
    parser = argparse.ArgumentParser()
    parser.add_argument("--rid", help="simple id for the robot")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('logger')

    RobotLogger(args.rid, ["youbot0", "youbot1"])
    rospy.spin()
