#!/usr/bin/python

PACKAGE = 'amr_navigation'
NODE = 'automated_tester'

import roslib
roslib.load_manifest(PACKAGE)
import rospy
import os

import sys
import argparse
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from actionlib import SimpleActionClient
from amr_msgs.msg import ExecutePathGoal, ExecutePathAction


def to_str(pose):
    rpy = euler_from_quaternion((pose.orientation.x, pose.orientation.y,
                                 pose.orientation.z, pose.orientation.w))
    return '%.3f %.3f %.3f' % (pose.position.x, pose.position.y, rpy[2])

def print_test_passed(expected,original):
    if(expected == original):
        print 'Test Passed'
    else:
        print 'Test Failed'
    pass

def feedback_cb(feedback):
    print ' -> %s %s' % ('visited' if feedback.reached else 'skipped',
                to_str(feedback.pose.pose))
    way_point_reached.append([True if feedback.reached else False])
    pass

def done_cb(status, result):
    print ''
    print 'Result'
    print '------'

    if result:
        for i, p, v in zip(range(1, len(poses) + 1), goal.path.poses,
                           result.visited):
            print '%2i) %s %s' % (i, 'visited' if v else 'skipped',
                                  to_str(p.pose))
    print ''

    state = ['pending', 'active', 'preempted', 'succeeded', 'aborted',
             'rejected', 'preempting', 'recalling', 'recalled',
             'lost'][status]
    print 'Action completed, state:', state
    

    for i in xrange(len(way_point_reached)):
        print_test_passed(way_points_result[i],way_point_reached[i])
        
    # rospy.signal_shutdown('Path execution completed')
    pass


if __name__ == '__main__':
    # We need this for command line arguments
    argv = sys.argv[1:]
    
    # Left wall following or Right wall following 
    wall = 0
    skip_unreachable = 1;

    # path to test files
    path = roslib.packages.get_pkg_dir("amr_navigation", required=True) + "/../evaluation/path_executor_test_cases/"
    test_cases = os.listdir(path);

    rospy.init_node(NODE, anonymous=True)
    SERVER = '/path_executor/execute_path'
    ep_client = SimpleActionClient(SERVER, ExecutePathAction)
    print 'Connecting to [%s] server...' % SERVER
    ep_client.wait_for_server()

    for test_file in test_cases:
        # File name
        # correct file name = "*.pth"
        execution_time = 15
        if(not(test_file.find('.pth') >= 0) or (test_file.find('~') >= 0) ):
            continue
        print "file: "+test_file
        # Load test case parameters from file:
        # Loading waypoints
        way_points = []
        way_points_result = []
        with open(path + test_file) as way_point_file:
            line_count = 0
            results = False
            for line in way_point_file:
                if(line.find('skip') >= 0):
                    flag = line.split('=')
                    skip_unreachable = int(flag[1])
                    continue

                if(line.find('time') >= 0):
                    flag = line.split('=')
                    execution_time = int(flag[1])
                    continue

                if(line.find('--end') == 0):
                    results = True
                    continue
                if results:
                    way_points_result.append(bool(int(line)))
                else:
                    numbers = line.split()
                    pose = []
                    for num in numbers:
                        try:
                            pose.append(float(num));
                        except ValueError:
                            print 'Unable to convert to float: '+num; 
                        
                    way_points.append(pose);
                    line_count = line_count + 1;

        # Preparing data to publish to action client
        string = ""
        for a in way_points:
            string = string + "".join(str(a))[1:-1] + "\n"
        
        way_point_reached = []
        poses = []
        goal = ExecutePathGoal()

        for i,pose in enumerate(way_points):
            try:
                x, y, yaw = pose;
            except ValueError:
                print_test_passed(way_points_result[i],False);
                continue;
            p = PoseStamped()
            q = quaternion_from_euler(0, 0, yaw)
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]
            goal.path.poses.append(p)
            poses.append((x, y, yaw))
        goal.skip_unreachable = skip_unreachable
        
        print ''
        print 'Goal'
        print '----'
        print 'Poses:'
        for i, p in enumerate(goal.path.poses):
            print '%2i) %s' % (i + 1, to_str(p.pose))
        print 'Skip unreachable:', skip_unreachable
        ep_client.send_goal(goal, done_cb=done_cb, feedback_cb=feedback_cb)
        rospy.sleep(execution_time);
        pass