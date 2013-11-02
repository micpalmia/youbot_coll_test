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

class RobotLogger

    _rob_self = "youbot0"
    _rob_instance = {}
    _rob_timing = {}

    def setup():
        rospy.init_node('logger')
        global prolog
        prolog = json_prolog.Prolog("json_prolog")

        # for each robot in the robot list
        for robot in [x for x in list if x != rob_self]:
            timing[robot] = 0.00
            rospy.Subscriber("/" + robot + "/odom" , Odometry, position_callback(robot))
            rospy.Subscriber("/" + robot + "/status" , Status, status_callback(robot))
        rospy.spin()

def position_callback(s):
    return partial(position_forwarder, sender=s)

def position_forwarder(data, sender):
    if current_milli_time() - timing[sender] > 10000:
        q = quaternionToMatrix(data.pose.pose);
        if not sender in instance:
          query = prolog.query("rdf_instance_from_class(move:'YouBot', O), rdf_assert(O, move:'hasRobotId', " + sender + "), rdf_assert(O, move:'isInMovingStatus', move:'Still')")
          inst = query.solutions().next()['O']
          query.finish()

          instance[sender] = inst
          rospy.loginfo("[" + rob_self + "] knows " + sender + " exists.")
        
        qry = "create_perception_instance(['ProgrammedPerception'], Perception), "\
                "set_object_perception('" + instance[sender] + "', Perception), "\
                "set_perception_pose(Perception, " + str(q) + ")"

        qry = prolog.query(qry)
        qry.finish()
        timing[sender] = current_milli_time()
        rospy.loginfo("[" + rob_self + "] Got position info for " + sender + ".")

def status_callback(s):
    return partial(status_forwarder, sender=s)

def status_forwarder(data, sender):
    print "Received satatus change: " + sender + "-" + data.status
    print instance[sender]
    query = prolog.query("rdf_retractall('" + instance[sender] + "', move:'isInMovingStatus', X), rdf_assert('" + instance[sender] + "', move:'isInMovingStatus', move:'" + data.status + "')")
    print str(query)
    res = query.solutions()
    query.finish

    status_update = rospy.ServiceProxy('move_coordinator/status_update', Empty)
    status_update()

current_milli_time = lambda: int(round(time.time() * 1000))

def quaternionToMatrix(pose):
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


if __name__ == '__main__':
    # get the namespace for the robot to be operated
    parser = argparse.ArgumentParser()
    parser.add_argument("--ns", help="namespace in which the robot s running")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('logger')

    global rob_self
    rob_self = args.ns
    setup()
