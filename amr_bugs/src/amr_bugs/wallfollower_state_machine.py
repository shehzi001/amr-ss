#!/usr/bin/env python

"""
This module provides a single construct() function which produces a Smach state
machine that implements wallfollowing behavior.

The constructed state machine has three attached methods:
    * set_ranges(ranges): this function should be called to update the range
                          readings
    * get_twist(): returns a twist message that could be directly passed to the
                   velocity publisher
    * set_config(config): updates the machine userdata with the new config

The constructed state machine is preemptable, i.e. each state checks whether
a preemption is requested and returns 'preempted' if that is the case.
"""

PACKAGE = 'amr_bugs'

import roslib
roslib.load_manifest(PACKAGE)
import smach
from preemptable_state import PreemptableState
from math import copysign, sqrt, radians, cos
from types import MethodType
from geometry_msgs.msg import Twist

import rospy


__all__ = ['construct']

# Aligns the vehicle to the wall.
def align_to_wall(ud):
    if abs(ud.side_difference) < ud.aligned_limit and ud.side_min < 1.0:
        return 'follow_wall'
        
    ud.velocity = (0, 0, ud.speed_max)
    pass

# Moves in a spiral to find an object with its side.
def find_wall(ud):
    # Switches to state align to wall, if the vehicle is not aligned to wall.
    if abs(ud.side_difference) >= ud.aligned_limit:
        return 'align_to_wall'
    
    # Switches to state follow wall, if the range is lower than maximum. That means there is an object.
    if ud.side_min < ud.range_max:
        return 'follow_wall'
    
    # Angular Velocity is reduced to make the spiral bigger.
    ud.spiral_speed = max(ud.spiral_speed - 0.001, 0.1)
    ud.velocity = (ud.speed_max, 0, ud.spiral_speed)
    pass

# Follows a wall.
def follow_wall(ud):
    # Switches to find wall. if range is maximum. That means there is no object to follow.
    if ud.side_min >= ud.range_max:
        ud.spiral_speed = ud.spiral_speed_max
        return 'find_wall'
    
    # The sensor used to detect obstacle in front of the vehicle is at 30 degrees and the distance to
    # the middle sensor is 60 degrees. If the vehicle follows the wall, than this sensor should have 
    # the range: clearance/cos(60). If that is smaller, than there is and object in front of the vehicle.
    front_clearance = ud.clearance / cos(radians(60))
    
    # Error values for the P-Controller are calculated.
    error_angle = ud.side_difference
    error_distance = ud.clearance - ud.side_min
    error_edge = max(front_clearance - ud.range_front, -0.01)
    
    # Sum of the angle error
    ud.error_angle_sum = min(max(ud.error_angle_sum + error_angle + error_edge, -3), 3)
    
    # P-Controllers for all speeds and speeds are limites, angular speed has also an I-Part
    angular_speed = min(max(0.5 * error_angle + ud.error_angle_sum * 0.03 + 2.0 * error_edge, -ud.speed_max), ud.speed_max)
    side_speed = min(max(1.0 * error_distance, -ud.speed_max), ud.speed_max) 
    # Forward speed gets reduces by the size of angular speed, to make better turns.
    forward_speed = min(max(ud.speed_max - abs(angular_speed), 0), ud.speed_max)
       
    ud.velocity = (forward_speed, side_speed, angular_speed)
    pass

#==============================================================================

def set_ranges(self, ranges):
    """
    This function will be attached to the constructed wallfollower machine.
    Its argument is a list of Range messages as received by a sonar callback.
    """
    # Switches the sensors based on which side is used to follow a wall.
    if self.userdata.mode == 0:
        self.userdata.full_side_min = min(ranges[0].range, ranges[1].range, ranges[14].range, ranges[15].range)
        self.userdata.side_min = min(ranges[0].range, ranges[15].range)
        self.userdata.side_difference = ranges[15].range - ranges[0].range
        self.userdata.range_front = ranges[2].range
    elif self.userdata.mode == 1:
        self.userdata.full_side_min = min(ranges[6].range, ranges[7].range, ranges[8].range, ranges[9].range)
        self.userdata.side_min = min(ranges[7].range, ranges[8].range)
        self.userdata.side_difference = ranges[8].range - ranges[7].range
        self.userdata.range_front = ranges[5].range

def get_twist(self):
    """
    This function will be attached to the constructed wallfollower machine.
    It creates a Twist message that could be directly published by a velocity
    publisher. The values for the velocity components are fetched from the
    machine userdata.
    """
    twist = Twist()
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    
    # Switches the sign of the output based on which side is used to follow a wall.
    if self.userdata.mode == 0:
        twist.linear.x = self.userdata.velocity[0]
        twist.linear.y = -self.userdata.velocity[1]
        twist.angular.z = -self.userdata.velocity[2]
    elif self.userdata.mode == 1:
        twist.linear.x = self.userdata.velocity[0]
        twist.linear.y = self.userdata.velocity[1]
        twist.angular.z = self.userdata.velocity[2]
        
    return twist


def set_config(self, config):
    """
    This function will be attached to the constructed wallfollower machine.
    It updates the relevant fields in the machine userdata.
    Its argument is the config object that comes from ROS dynamic reconfigure
    client.
    """
    self.userdata.mode = config['mode']
    self.userdata.clearance = config['clearance']
    return config


def construct():
    sm = smach.StateMachine(outcomes=['preempted'])
    # Attach helper functions
    sm.set_ranges = MethodType(set_ranges, sm, sm.__class__)
    sm.get_twist = MethodType(get_twist, sm, sm.__class__)
    sm.set_config = MethodType(set_config, sm, sm.__class__)    
    # Set initial values in userdata
    sm.userdata.velocity = (0, 0, 0)
    sm.userdata.mode = 1
    sm.userdata.clearance = 0.5
    sm.userdata.spiral_speed_max = 0.7
    sm.userdata.spiral_speed = sm.userdata.spiral_speed_max    
    sm.userdata.full_side_min = 0
    sm.userdata.side_min = 0
    sm.userdata.side_difference = 0
    sm.userdata.range_front = 0
    sm.userdata.range_max = 5.0
    sm.userdata.speed_max = 0.5
    sm.userdata.aligned_limit = 0.2
    sm.userdata.error_angle_sum = 0
    
    # Add states
    with sm:
        smach.StateMachine.add('ALIGN_TO_WALL',
                               PreemptableState(align_to_wall,
                                                input_keys=['side_min', 'range_max', 'side_difference', 'speed_max', 'aligned_limit'],
                                                output_keys=['velocity'],
                                                outcomes=['follow_wall', 'find_wall']),
                               transitions={'follow_wall': 'FOLLOW_WALL',
                                            'find_wall': 'FIND_WALL'})
        
        smach.StateMachine.add('FIND_WALL',
                               PreemptableState(find_wall,
                                                input_keys=['speed_max', 'side_min', 'side_difference', 'range_max', 'spiral_speed', 'aligned_limit'],
                                                output_keys=['velocity', 'spiral_speed'],
                                                outcomes=['follow_wall', 'align_to_wall']),
                               transitions={'follow_wall': 'FOLLOW_WALL',
                                            'align_to_wall': 'ALIGN_TO_WALL'})
        
        smach.StateMachine.add('FOLLOW_WALL',
                               PreemptableState(follow_wall,
                                                input_keys=['error_angle_sum', 'speed_max', 'side_difference', 'range_max', 'range_front', 'side_min', 'clearance', 'spiral_speed', 'spiral_speed_max'],
                                                output_keys=['velocity', 'spiral_speed', 'error_angle_sum'],
                                                outcomes=['find_wall']),
                               transitions={'find_wall': 'FIND_WALL'})
    return sm
