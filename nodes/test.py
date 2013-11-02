#!/usr/bin/env python
 
import roslib; roslib.load_manifest('json_prolog')
 
import rospy
import json_prolog

import actionlib
import move_base_msgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from standard_srvs.srv import Empty

rob_self = "youbot0"
 
if __name__ == '__main__':
    rospy.init_node('executor')
    prolog = json_prolog.Prolog()

    # TODO should be requested from command line
    query = prolog.query("plan_subevents(move:'MovementTest', SEs)")
    SEs = query.solutions().next()['SEs']

    query.finish()

    for event in SEs:
        query = prolog.query("class_properties('" + event + "', move:'providedByMotionPrimitive', P), class_properties(P, move:'providedByROSAction', A)")
        raw_action = query.solutions().next()
        query.finish()
        action = raw_action['A']['term'][1]['term'][2]

        # TODO do something if there's more than one primitive

        if action == 'move_base':
            query = prolog.query("class_properties('" + event + "', move:'destXValue', X), class_properties('" + event + "', move:'destYValue', Y)")
            coords = query.solutions().next()
            query.finish()

            coordinate_move_base(float(coords['X']['term'][1]['term'][2]), float(coords['Y']['term'][1]['term'][2]))

def coordinate_move_base(x, y):
    rospy.Service('move_status_update', Empty, handle_status_update)
    pub = rospy.Publisher('status', std_msgs.msg.String)
    pub.publish("Moving")
    
    solutions = prolog.query("rdf(R, move:'isInMovingStatus', move:'Moving')").solutions()
    mov_robots = []

    try:
        while True:
            mov_robots.append(solutions.next())
    except StopIteration:   
        pass

    priority = true
    for r in mov_robots
        if int(rob_self[-1:]) > int(r[-1:]):
            priority = false 

    if len(mov_robots) == 0:

        client = actionlib.SimpleActionClient('/' + rob_self + '/move_base', move_base_msgs.msg.MoveBaseAction)
        client.wait_for_server()
        
        goal = move_base_msgs.msg.MoveBaseGoal()
        goal.target_pose.header.frame_id = "/" + rob_self + "/base_link"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.orientation.w = 1.0;

        client.send_goal(goal)
        client.wait_for_result()
        # client.get_result()
