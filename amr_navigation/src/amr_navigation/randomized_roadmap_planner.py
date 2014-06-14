#!/usr/bin/env python

import rospy
from math import sqrt
from random import uniform
from pygraph.classes.graph import graph
from pygraph.classes.exceptions import NodeUnreachable
from pygraph.algorithms.heuristics.euclidean import euclidean
from pygraph.algorithms.minmax import heuristic_search

class RandomizedRoadmapPlanner:

    def __init__(self, point_free_cb, line_free_cb, dimensions):
        """
        Construct a randomized roadmap planner.

        'point_free_cb' is a function that accepts a point (two-tuple) and
        outputs a boolean value indicating whether the point is in free space.

        'line_free_cb' is a function that accepts two points (the start and the
        end of a line segment) and outputs a boolen value indicating whether
        the line segment is free from obstacles.

        'dimensions' is a tuple of tuples that define the x and y dimensions of
        the world, e.g. ((-8, 8), (-8, 8)). It should be used when generating
        random points.
        """
        self.point_free_cb = point_free_cb
        self.line_free_cb = line_free_cb
        self.dimensions = dimensions
        
        # Instantiate graph, heuristic and maximum number of tries
        self.graph = graph()
        self.heuristic = euclidean()
        self.max_tries = 50
        pass
        
    def plan(self, point1, point2):
        """
        Plan a path which connects the two given 2D points.

        The points are represented by tuples of two numbers (x, y).

        Return a list of tuples where each tuple represents a point in the
        planned path, the first point is the start point, and the last point is
        the end point. If the planning algorithm failed the returned list
        should be empty.
        """
        path_to_target = list()
        found_path_to_target = False
        
        # Saves start and end identifiers
        start_identifier = len(self.graph.nodes())
        end_identifier = start_identifier + 1
        
        #Check if points are free
        if self.point_free_cb(point1) and self.point_free_cb(point2):
            self.graph.add_node(start_identifier, attrs=[('position', point1)])
            self.graph.add_node(end_identifier, attrs=[('position', point2)])
            # Checks if start and end point can be connected
            if self.line_free_cb(point1, point2):
                self.graph.add_edge((start_identifier, end_identifier), wt = self.distance(point1, point2))
            # Find edges to the rest of the graph
            for node_identifier, attr in self.graph.node_attr.iteritems():
                position = attr[0][1]
                if point1 != position and point2 != position: # if point and position is the same, line_free does not return and hangs
                    if self.line_free_cb(point1, position):
                        self.graph.add_edge((start_identifier, node_identifier), wt = self.distance(point1, position)) 
                    if self.line_free_cb(point2, position):
                        self.graph.add_edge((end_identifier, node_identifier), wt = self.distance(point2, position))
        else:
            # Stops if start or end point are not free
            rospy.logwarn("Start or End Point are not free")
            return path_to_target
        
        # Searches for a path and if there is none, a random point is created and added to the graph
        count_tries = 0
        while not found_path_to_target:
            try:
                # Check if there is a path
                self.heuristic.optimize(self.graph)
                identifier_path = heuristic_search(self.graph, start_identifier, end_identifier, self.heuristic)
                found_path_to_target = True
                rospy.logwarn("Found a Path")
                # Resolve identfier and push them into path_to_target
                for identifier in identifier_path:
                    node_pose = self.graph.node_attributes(identifier)
                    path_to_target.append(node_pose[0][1])                    
            except NodeUnreachable:
                # Create a new random point 
                random_point_x = uniform(self.dimensions[0][0],self.dimensions[0][1])
                random_point_y = uniform(self.dimensions[1][0],self.dimensions[1][1])
                random_point = (random_point_x, random_point_y)
                # If new random point is free add it as node to the graph
                if self.point_free_cb(random_point):
                    identifier = len(self.graph.nodes())
                    self.graph.add_node(identifier, attrs=[('position', random_point)])
                    # Check if new random point can be connected to any other point and add edges
                    for node_identifier, attr in self.graph.node_attr.iteritems():
                        position = attr[0][1]
                        if random_point != position:
                            if self.line_free_cb(random_point, position):
                                self.graph.add_edge((identifier, node_identifier), wt = self.distance(random_point, position))                                
            # Check if max tries are exceeded and stop
            count_tries += 1
            if count_tries >= self.max_tries:
                rospy.logwarn("Maximum tries exceeded, no Path could be found")
                break
                                
        return path_to_target

    def distance(self, point1, point2):
        '''
        Calculates the distance between two points.
        '''
        return sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2))

    def remove_edge(self, point1, point2):
        """
        Remove the edge of the graph that connects the two given 2D points.

        The points are represented by tuples of two numbers (x, y).

        Has an effect only if both points have a corresponding node in the
        graph and if those nodes are connected by an edge.
        """
        pass

#==============================================================================
