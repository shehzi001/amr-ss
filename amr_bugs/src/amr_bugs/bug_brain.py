#!/usr/bin/env python

PACKAGE = 'amr_bugs'

import rospy
import math
import planar
from planar import Point, Vec2, EPSILON
from planar.c import Line
from math import degrees


class BugBrain:

    # declearing constants
    LEFT_WALLFOLLOWING = 0
    RIGHT_WALLFOLLOWING = 1
    LEFT_SIDE = -1
    RIGHT_SIDE = 1

    def __init__(self, goal_x, goal_y, side):
        # saving which wall following is being used.
        self.wall_side = side
        # saving goal point
        self.wp_destination = Point(goal_x, goal_y)
        # flag to check that robot has started wall following.
        self.path_started = False
        #the tolerence == planar.EPSILON at default value does not work good
        planar.set_epsilon(0.2)
        # storing distance to the destination when leaving the wall 
        self.distance_when_left = 9999 # huge initial value
        pass
    
    # method to determin if the destenation is on opposit side of wall being followed.
    # @param: distance
    #         signed distance from the robot to goal.
    #         can be obtained by path_line.distance_to(ROBOT CURRENT POSITION).
    def is_destination_opposite_to_wall(self,distance):
        direction = math.copysign(1,distance)

        if(self.wall_side == self.LEFT_WALLFOLLOWING):
            if(direction == self.RIGHT_SIDE):
                return True
            else:
                return False
        else:
            if(direction == self.LEFT_SIDE):
                return True
            else:
                return False
        pass
            
    def follow_wall(self, x, y, theta):
        """
        This function is called when the state machine enters the wallfollower
        state.
        """
        # compute and store necessary variables
        theta = degrees(theta)
        position = Point(x,y)
        # storing distance to goal, later using it to decide when to leave the wall
        self.distance_when_left = self.wp_destination.distance_to(Point(x,y))

        self.ln_path = Line.from_points([position,self.wp_destination])
        # saving where it started wall following
        self.wp_wf_start = position
        pass
        

    def leave_wall(self, x, y, theta):
        """
        This function is called when the state machine leaves the wallfollower
        state.
        """
        # compute and store necessary variables
        self.path_started = False
        self.distance_when_left = self.wp_destination.distance_to(Point(x,y))
        self.wp_left_wall_at = Point(x,y)
        pass

    def is_goal_unreachable(self, x, y, theta):
        """
        This function is regularly called from the wallfollower state to check
        the brain's belief about whether the goal is unreachable.
        """
        # if the robot goes around an obstacle and
        # reaches the starting point and the destenation is still not reached then
        # the goal is unreachable.
        distance_to_path= self.ln_path.distance_to(Point(x,y))

        if(abs(distance_to_path) < planar.EPSILON and
           Vec2(x,y).almost_equals(self.wp_wf_start) and 
           self.path_started):
            rospy.logwarn("UNREACHABLE POINT!")
            return True

        return False

    def is_time_to_leave_wall(self, x, y, theta):
        """
        This function is regularly called from the wallfollower state to check
        the brain's belief about whether it is the right time (or place) to
        leave the wall and move straight to the goal.
        """

        self.current_theta  = degrees(theta)

        self.wp_current_position = Point(x,y)
        self.current_direction = Vec2.polar(angle = self.current_theta,length = 1)
        #Robot Orientation Line.
        self.ln_current_orentation = Line(Vec2(x,y),self.current_direction)

        # the prependicular line to the path
        self.ln_distance = self.ln_path.perpendicular(self.wp_current_position)
        
        
        distance_to_path= self.ln_path.distance_to(Point(x,y))
        self.distance_to_path = distance_to_path
        distance_to_destination = self.ln_current_orentation.distance_to(self.wp_destination)
        if(abs(distance_to_path) > 0.5):
            self.path_started =True

        self.distance_to_goal = self.wp_destination.distance_to(Point(x,y))
        """
        checking if distance to the straight path is approx. 0 and
        if destenation on the opposit side of wall then leave the path
        NOTE and TODO: works only for the circles not for complex path.
        """
        if(abs(distance_to_path) < planar.EPSILON and 
           self.distance_to_goal < self.distance_when_left and
           self.is_destination_opposite_to_wall(distance_to_destination) and 
           self.path_started): # is robot started following wall!
            self.wp_wf_stop = Point(x,y)
            return True

        return False
