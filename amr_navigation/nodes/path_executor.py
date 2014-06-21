#!/usr/bin/env python

PACKAGE = 'amr_navigation'
NODE = 'path_executor'

import roslib
roslib.load_manifest(PACKAGE)
import rospy

from smach_msgs.msg import SmachContainerStatus
from actionlib import SimpleActionClient, SimpleActionServer
from nav_msgs.msg import Path
from amr_msgs.msg import MoveToAction, MoveToGoal, ExecutePathAction, \
                         ExecutePathFeedback, ExecutePathResult


class PathExecutor:

    SLEEP_RATE = 5
    GOAL_SUCCEEDED = 3
    GOAL_ABORTED = 4  

    def __init__(self, move_to_topic, execute_path_topic, path_publisher_topic, wall_follower_topic, timeout):
        '''
        Initialized all needed variables and starts the client, server and publisher.
        '''
        self.goal = None
        self.waypoint_toggle = False
        self.path_index = 0
        self.waypoint_unreachable = False
        self.loop_rate = rospy.Rate(PathExecutor.SLEEP_RATE)
        self.timeout = rospy.Duration(timeout)
        self.timeout_val = timeout
        self.timer =None;

        self.client = SimpleActionClient(move_to_topic, MoveToAction)
        self.client.wait_for_server()
        self.publisher = rospy.Publisher(path_publisher_topic, Path)
        if not wall_follower_topic == None:
            self.wall_follower = rospy.Subscriber(wall_follower_topic, SmachContainerStatus, self.wall_follower_cb)
        self.server = SimpleActionServer(execute_path_topic, ExecutePathAction, self.execute_cb, False)
        self.server.start()
        
        rospy.loginfo("Path executor ready with timout : {0}".format(self.timeout_val));
        pass

    def execute_cb(self, action_path):
        '''
        Is called when a new path is send to this node. Each waypoint is then 
        send to a motion controller if the previous was reached or is unreachable. 
        '''
        rospy.loginfo("Received new path")
        self.publisher.publish(action_path.path)
        
        self.result = ExecutePathResult()
        [self.result.visited.append(0) for pose in action_path.path.poses]
        
        self.path_index = 0
        self.goal = MoveToGoal()
        self.goal.target_pose = action_path.path.poses[self.path_index] 
        self.client.send_goal(self.goal, done_cb=self.move_to_done_cb)
        
        while not rospy.is_shutdown():
            
            # Checks if preemption was requested and if yes, than preempts the path executor.
            if self.server.is_preempt_requested():
                self.server.set_preempted(self.result)
                rospy.logwarn("Preempted path execution")
                break
            
            # Waypoint_toggle is set by move_to_done_cb and signalizes that a waypoint was reached or is unreachable.
            if self.waypoint_toggle:
                self.waypoint_toggle = False
                self.path_index += 1
                
                # If waypoint is unreachable and it's not allowed to skip a waypoint, path executor aborts.
                if self.waypoint_unreachable and not action_path.skip_unreachable:
                    self.server.set_aborted(self.result)
                    rospy.logwarn("Aborted path execution")
                    break
                    pass
                
                # Send the next waypoint to a motion controller. If there is no next waypoint, path executor succeeded.
                if self.path_index < len(action_path.path.poses):
                    self.goal = MoveToGoal()
                    self.goal.target_pose = action_path.path.poses[self.path_index]
                    self.client.send_goal(self.goal, done_cb=self.move_to_done_cb)
                else:
                    self.server.set_succeeded(self.result)
                    rospy.logwarn("Succeeded path execution")
                    break
                
            self.loop_rate.sleep()
        pass
        
    def move_to_done_cb(self, state, result):
        '''
        Is called when a waypoint is reached. Toggles a waypoint_toggle to
        signalize the execute_cb to send the next waypoint to a motion controller.
        '''
        feedback = ExecutePathFeedback()
        feedback.pose = self.goal.target_pose
        
        # Evaluates the state
        if state == PathExecutor.GOAL_SUCCEEDED:
            rospy.loginfo("Waypoint [%.2f, %.2f] reached", self.goal.target_pose.pose.position.x, self.goal.target_pose.pose.position.y)
            feedback.reached = True
        elif state == PathExecutor.GOAL_ABORTED:
            rospy.loginfo("Waypoint [%.2f, %.2f] unreachable", self.goal.target_pose.pose.position.x, self.goal.target_pose.pose.position.y)
            feedback.reached = False
            self.waypoint_unreachable = True
        
        self.result.visited[self.path_index] = feedback.reached
        self.server.publish_feedback(feedback)
        self.waypoint_toggle = True
        pass
    def wall_follower_cb(self, data):
        self.wallfollower_current_state = data.active_states[0]
        if(self.wallfollower_current_state == 'FOLLOW_WALL'):
            if self.is_timeout_available():
                self.timer = rospy.Timer(self.timeout, self.timer_cb, oneshot=True);
        elif(self.wallfollower_current_state == 'None'):
            if not(self.timer == None):
                self.timer.shutdown();
        pass

    def is_timeout_available(self):
        return not(self.timeout_val == 0)
        pass
    def timer_cb(self,event):
        rospy.logwarn("goal premted due to timeout")
        if(self.wallfollower_current_state == 'FOLLOW_WALL'):
            self.waypoint_toggle = True
            self.waypoint_unreachable = True
            if not(self.timer == None):
                self.timer.shutdown();

if __name__ == '__main__':
    rospy.init_node(NODE)
    move_to_topic = '/motion_controller/move_to'
    execute_path_topic = '/path_executor/execute_path'
    path_publisher_topic = '/path_executor/current_path'
    obstacle_avoidance_timeout = 0;
    wall_follower_topic = None
    if rospy.get_param('~use_obstacle_avoidance', True):
        move_to_topic = '/bug2/move_to'
        wall_follower_topic = '/smach_inspector/smach/container_status'
        if rospy.has_param('~obstacle_avoidance_timeout'):
            obstacle_avoidance_timeout = rospy.get_param('~obstacle_avoidance_timeout')
    
    pe = PathExecutor(move_to_topic, execute_path_topic, path_publisher_topic,wall_follower_topic,obstacle_avoidance_timeout)
    rospy.spin()
    pass
