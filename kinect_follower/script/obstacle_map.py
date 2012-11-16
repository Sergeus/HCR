#!/usr/bin/env python

import roslib; roslib.load_manifest('kinect_follower')
import rospy

from sensor_msgs.msg import LaserScan, PointCloud


from multiprocessing import Lock
import time

#numpy and scipy imports
import numpy as np
from scipy import weave
from scipy.weave import converters

#our KD Tree
from scipy.spatial import cKDTree

from numpy import sin, cos, floor, around, array, ceil, fabs, sign
import matplotlib.pyplot as plt


from robot_profile import RobotProfile

class ObstacleMap:
    def __init__(self, robot, lookahead_distance = 4.0):

        #set the robot
        if robot is None:
            print "Error! I need a robot profile to work"
            return

        self.robot = robot

        #how far ahead does an obstacle have to be to not matter?
        self.lookahead_distance = lookahead_distance


        #initialisations
        self.obstacles = []
        self.obstacles_in_memory = []
        self.obstacles_tree = None
        self.obstacles_lock = Lock()

        return


    def addObstacles(self, obstacles=None):
        '''
        Adds obstacles to internal structures
        @param obstacles: a list of obstacles in body frame coordinates
        @return: nothing
        '''

        #check if there is anything to add
        if obstacles is None:
            return

        #first we acquire a lock
        self.obstacles_lock.acquire()

        #inititlisations
        self.obstacles = []
        temp_obstacles = []


        #reduce the space of obstacles
        for obs in obstacles:
            if np.linalg.norm(obs) < self.lookahead_distance:
                temp_obstacles.append(obs)

        self.obstacles_in_memory = temp_obstacles
        start_time = time.time()
        if len(temp_obstacles) <= 1:
            #not enough samples for a KD-Tree, just use a plain list
            self.obstacles_tree = None
            self.obstacles = temp_obstacles
        else:
            #we have enough samples to build a KDtree
            self.obstacles_tree = cKDTree(np.array(obstacles))
            #print the KDTree creation time
            #print "KDTree creation time: ", time.time() - start_time

        #release the lock
        self.obstacles_lock.release()
        return


    def inRobot(self, pt):
        '''
        Checks if the point is in the robot
        Assumes that both robot polygon and robot are in the same frame
        @param pt: the point
        @return: true if point is in the robot
        '''
        return self.isPointInPoly(pt, self.robot.footprint)

    def checkForCollisionAt(self, position):
        '''
        Checks if a particular location is in collision with an obstacle
        @param robot_pos: the robot's (center) position
        @return: whether the position is in collision with an obstacle
        '''
        #first we compute the footprint using the position
        footprint = self.getFootprintAt(position)
        if self.obstacles_tree is None:
            #we don't have a tree, only a list
            if len(self.obstacles) == 0:
                return False
            else:

                #check whether the obstacles fall into the polygon
                for pt in self.obstacles:
                    if self.isPointInPoly(pt, footprint.tolist()):
                        return True

                return False
        else:
            #we have a tree
            self.obstacles_lock.acquire()
            #st = time.time()
            k = len(self.obstacles_tree.data)
            potential_obstacles = self.obstacles_tree.query(position[0:2], k=k,
                distance_upper_bound = self.robot.outer_radius + 0.01, eps=0.01)
            #print "KD Tree time: ", time.time() - st
            #print len(potential_obstacles)
            self.obstacles_lock.release()
            valids = np.isinf(potential_obstacles[0])

            #if no points in the radius
            if np.sum(valids) >= k:
                return False

        #st = time.time()
        max_items = len(self.obstacles_tree.data)
        keys = valids == False
        for pt_index in potential_obstacles[1][keys]:

            if pt_index >= max_items: continue
            pt = self.obstacles_tree.data[pt_index]

            if self.isPointInPoly(pt.tolist(), footprint.tolist()):
                #print "Manual Checking time: ", time.time() - st
                return True

        #print "Manual Checking time: ", time.time() - st
        return False

    def checkForCollisionAt_Rough(self, position):
        '''
        Checks if a particular location is in collision with an obstacle
        @param robot_pos: the robot's (center) position
        @return: whether the position is in collision with an obstacle
        '''
        #first we compute the footprint using the position
        footprint = self.getFootprintAt(position)

        if self.obstacles_tree is None:
            #we don't have a tree, only a list
            if len(self.obstacles) == 0:
                return False
            else:

                #check whether the obstacles fall into the polygon
                for pt in self.obstacles:
                    if self.isPointInPoly(pt, footprint.tolist()):
                        return True

                return False
        else:
            #we have a tree
            self.obstacles_lock.acquire()
            #st = time.time()
            k = len(self.obstacles_tree.data)
            potential_obstacles = self.obstacles_tree.query(position[0:2], k=k,
                distance_upper_bound = self.robot.outer_radius + 0.01, eps=0.01)
            #print "KD Tree time: ", time.time() - st
            #print len(potential_obstacles)
            self.obstacles_lock.release()
            valids = np.isinf(potential_obstacles[0])

            #if no points in the radius
            if np.sum(valids) >= k:
                return False
            else:
                return True



    def getFootprintAt(self, position):
        """
        Computes a footprint using C code (using weave)
        @param position: robot position as a list
        @return:
        """
        orig_footprint = self.robot.footprint
        n = len(orig_footprint)
        footprint = []
        code = """
            for (int i=0; i<n; i++) {
                py::object pt = orig_footprint[i];
                py::list new_pt;
                double x = cos(position[2])* (double) pt[0] - sin(position[2])* (double) pt[1] + (double) position[0];
                double y = sin(position[2])* (double) pt[0] + cos(position[2])* (double) pt[1] + (double) position[1];
                new_pt.append(x);
                new_pt.append(y);
                footprint.append(new_pt);
            }
            return_val = footprint;
             """
        footprint = weave.inline(code, ['footprint', 'position', 'orig_footprint', 'n'], type_converters=converters.blitz)
        return array(footprint)



    def isPointInPoly(self, pt, poly):
        '''
        Checks if point pt is in a polygon poly
        @param pt: the point
        @param poly: the polygon
        @return: true if point is in the polygon
        '''


        n = len(poly)

        code = """
            double p1x, p2x, p1y, p2y;
            bool inside = false;

            double x,y;
            x = pt[0]; y = pt[1];

            py::object p1 = poly[0];
            p1x = p1[0];
            p1y = p1[1];
            for (int i=0; i<n+1; i++) {
                double p2x, p2y;
                py::object p2 = poly[i%n];
                p2x = p2[0];
                p2y = p2[1];
                double miny, maxy, minx, maxx;
                if (p1x < p2x) {
                    minx = p1x;
                    maxx = p2x;
                } else {
                    minx = p2x;
                    maxx = p1x;
                }

                if (p1y < p2y) {
                    miny = p1y;
                    maxy = p2y;
                } else {
                    miny = p2y;
                    maxy = p1y;
                }


                if ((y > miny) && (y <= maxy) && (x <= maxx)) {
                    double xinters = 0;
                    if (p1y != p2y) {
                        xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x;
                        if ((p1x == p2x) || (x <= xinters)) {
                            inside = !inside;
                        }
                    }
                }

                p1x = p2x;
                p1y = p2y;

            }
            return_val = inside; """
        in_poly = weave.inline(code, ['poly', 'pt', 'n'], type_converters=converters.blitz)
        return in_poly
