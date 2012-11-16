#!/usr/bin/env python

import roslib; roslib.load_manifest('kinect_follower')
import rospy

from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point, Twist, PoseStamped, Pose, PolygonStamped, Point32, PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

from multiprocessing import Lock
import time

import numpy as np
from numpy import sin, cos, floor, around, array, ceil, fabs

from scipy import weave
from scipy.weave import converters

from obstacle_map import ObstacleMap
from robot_profile import RobotProfile

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class SharedControl():

    def __init__(self, robot):
        #initliase the node
        rospy.init_node('shared_control')

        #set up obstacle map
        self.robot = robot
        self.obstacle_map = ObstacleMap(robot)

        #set up locks
        self.obstacle_map_lock = Lock()
        self.js_lock = Lock()
        self.laser_lock = Lock()
        self.sonar_lock = Lock()
        self.odom_lock = Lock()
        self.polar_range_hist_lock = Lock()

        #set up internal variables
        self.curr_cmd = [0.0, 0.0] #0 linear, 0 angular
        self.curr_vel = [0.0, 0.0] #not moving initially
        self.sonar_readings = []
        self.laser_readings = []
        self.angles = []
        self.cosangles = []
        self.sinangles = []

        #basic safeguarding
        self.max_vel = 0.5
        self.max_turn_vel = 0.5

        #Forward simulation parameters
        self.delay_time = 0.1   #max delay before command is issued
        self.time_applied = 0.2 #how long the command is applied
        self.time_decc = 0.5    #how long to compute deceleration
        self.sim_interval = 0.1 #simulation interval

        #VFH parameters
        self.prh_smooth_window = 2
        self.prh_resolution = 2.0/180.0*np.pi
        self.polar_range_hist = [0]*int(2.0*np.pi/self.prh_resolution)
        self.raw_polar_range_hist = [0]*int(2.0*np.pi/self.prh_resolution)
        self.polar_range_hist_lock = Lock()
        self.max_considered_dist = 2.0

        self.closeness_weight = 0.5
        self.free_zone_weight = 0.5

        self.turning_coeff    = 2.0


        #set up subscribers
        rospy.Subscriber('js_cmd_vel', Twist, self.cmdVelCallback)
        rospy.Subscriber('base_scan', LaserScan, self.laserCallback)
        rospy.Subscriber('sonar_pc', PointCloud, self.sonarCallback)
        rospy.Subscriber('odom', Odometry, self.odomCallback)


        #set up publishers
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist)
        self.obs_pub = rospy.Publisher('obstacle', Marker)
        self.polar_hist_pub = rospy.Publisher('polar_histogram', LaserScan)
        self.zone_score_pub = rospy.Publisher('zone_scores', LaserScan)
        self.projection_pub = rospy.Publisher('projection', PoseArray)
        self.time_taken_pub = rospy.Publisher('sc_time_taken', Float32)

        return

    #=============================================================================================
    # ROS Callbacks
    #=============================================================================================

    def cmdVelCallback(self, data):
        self.js_lock.acquire()

        #set the current requested command velocity
        self.curr_cmd = [data.linear.x, data.angular.z]

        self.js_lock.release()
        return

    def laserCallback(self, data):
        self.laser_lock.acquire()

        #initialise angles if not done yet
        if len(self.angles) != len(data.ranges):
            self.angles = []
            self.angles = np.arange(data.angle_min, data.angle_max+data.angle_increment, data.angle_increment)

        #calculate the cos and sin angles (caching)
        if len(self.cosangles) != len(self.angles):
            self.cosangles = [cos(angle) for angle in self.angles]
            self.sinangles = [sin(angle) for angle in self.angles]


        #set the points
        self.laser_header = data.header

        #warning: this transformation assumes that the laser is NOT rotated in any way
        #technically, we should use a tf transform for this (future work)
        self.laser_readings = [ [ data.ranges[i]*self.cosangles[i] + self.robot.laser_pos[0],
                                  data.ranges[i]*self.sinangles[i] + self.robot.laser_pos[1] ]
                                    for i in range(len(data.ranges)) ]

        self.laser_lock.release()
        return

    def sonarCallback(self, data):
        self.sonar_lock.acquire()
        #set the sonar readings
        self.sonar_readings = [ [pt.x, pt.y] for pt in data.points]

        self.sonar_lock.release()
        return


    def odomCallback(self, data):
        self.odom_lock.acquire()

        self.curr_vel = [data.twist.twist.linear.x, data.twist.twist.angular.z]

        self.odom_lock.release()


    #=============================================================================================
    # Obstacle Map
    #=============================================================================================

    def updateObstacleMap(self):
        """
        Updates the obstacle map
        """

        all_sensor_readings = self.laser_readings + self.sonar_readings

        #we remove all the sensor readings that occur inside the robot frame
        restricted_sensor_readings = []
        for pt in all_sensor_readings:
            if not self.obstacle_map.inRobot(pt):
                restricted_sensor_readings.append(pt)

        #add the obstacles to the obstacle map
        self.obstacle_map_lock.acquire()
        self.obstacle_map.addObstacles(restricted_sensor_readings)
        self.obstacle_map_lock.release()

        return

    #=============================================================================================
    # DWA Functions
    #=============================================================================================

    def checkForCollision(self, cmd, rough=False, publish=False):
        """
        Checks for collisions for a given command velocity
        @param: cmd the desired command
        @param: rough perform "rough" or point-in-polygon check?
        @param: publish publish the projection?
        @returns: True if this command results in a collision, False otherwise
        """
        #we first compute the expected trajectory
        trajectory = self.computeTrajectoryWithDecceleration(cmd, self.curr_vel, self.delay_time,
            self.time_applied, self.time_decc)

        if publish:
            self.publishProjection(trajectory)

        #then we check against the obstacle map for possible collisions
        first_pos = trajectory[0]
        if rough:
            for pos in trajectory:
                if np.linalg.norm(np.array(pos) - np.array(first_pos)) > 0.05:
                    if rough:
                        if self.obstacle_map.checkForCollisionAt_Rough(pos):
                            return True
                    else:
                        if self.obstacle_map.checkForCollisionAt(pos):
                            return True

        return False


    def computeTrajectoryWithDecceleration(self, cmd_vel, current_speed, delay_time, time_applied, time_decc):
        """
        computes the trajectory with the  cmd_vel applied for time_applied and then back to 0,0 for time_decc
        @param cmd_vel: the intended command velocities
        @param current_speed: the current speed
        @param time_applied: the time for which the current speed is applied
        @param time_decc: the time for which (0,0) is applied
        @return:
        """
        accs = [self.robot.acc_x, self.robot.acc_th, self.robot.dacc_x,self.robot.dacc_th ]

        #project forwards in time
        sim_interval = self.sim_interval
        sim_interval_sq = sim_interval*sim_interval
        cmd_vel = list(cmd_vel)
        current_speed = list(current_speed)

        max_acc_x = self.robot.max_acc_x
        max_acc_th = self.robot.max_acc_th
        max_decc_x = self.robot.max_dacc_x
        max_decc_th = self.robot.max_dacc_th


        code = """
            double x, y, th;
            x = y = th = 0;

            double vx, vth;
            vx = current_speed[0];
            vth = current_speed[1];

            double cmd_x = cmd_vel[0];
            double cmd_th = cmd_vel[1];

            //are we accelerating or decelerating?
            double acc_x;
            double acc_th;

            bool b_ax = false;
            if (cmd_x > vx) {
                acc_x = accs[0];
                b_ax = true;
            } else {
                acc_x = accs[2];
                acc_x = -1*acc_x;
            }

            bool b_ath = false;
            if (cmd_th > vth) {
                acc_th = accs[1];
                b_ath = true;
            } else {
                acc_th = -1* (double) accs[3];

            }

            py::list traj;

            //delay at the beginning
            for (double i=0; i<=delay_time; i+=sim_interval) {
                //check if we have already attained our velocity
                if (fabs(vx - (double) cmd_x) < 1e-6) vx = cmd_x;

                //check if we have exceeded our target
                if (b_ax && (vx > cmd_x )) vx = cmd_x;
                if (!b_ax && (vx < cmd_x )) vx = cmd_x;

                if (fabs(vth - (double) cmd_th) < 1e-6) vth = cmd_th;
                if (b_ath && (vth > cmd_th )) vth = cmd_th;
                if (!b_ath && (vth < cmd_th )) vth = cmd_th;

                x += vx*cos(th)*sim_interval;
                y += vx*sin(th)*sim_interval;
                th += vth*sim_interval;

                py::list new_pt;
                new_pt.append(x);
                new_pt.append(y);
                new_pt.append(th);
                traj.append(new_pt);
            }

            //application of the command
            for (double i=0; i<=time_applied; i+=sim_interval) {
                vx += acc_x*sim_interval;

                //check if we have already attained our velocity
                if (fabs(vx - (double) cmd_x) < 1e-6) vx = cmd_x;

                //check if we have exceeded our target
                if (b_ax && (vx > cmd_x )) vx = cmd_x;
                if (!b_ax && (vx < cmd_x )) vx = cmd_x;

                vth += acc_th*sim_interval;

                if (fabs(vth - (double) cmd_th) < 1e-6) vth = cmd_th;
                if (b_ath && (vth > cmd_th )) vth = cmd_th;
                if (!b_ath && (vth < cmd_th )) vth = cmd_th;

                x += vx*cos(th)*sim_interval;
                y += vx*sin(th)*sim_interval;
                th += vth*sim_interval;

                py::list new_pt;
                new_pt.append(x);
                new_pt.append(y);
                new_pt.append(th);
                traj.append(new_pt);
            }

            //delay before decellerating
            for (double i=0; i<=delay_time; i+=sim_interval) {
                //check if we have already attained our velocity
                if (fabs(vx - (double) cmd_x) < 1e-6) vx = cmd_x;

                //check if we have exceeded our target
                if (b_ax && (vx > cmd_x )) vx = cmd_x;
                if (!b_ax && (vx < cmd_x )) vx = cmd_x;

                if (fabs(vth - (double) cmd_th) < 1e-6) vth = cmd_th;
                if (b_ath && (vth > cmd_th )) vth = cmd_th;
                if (!b_ath && (vth < cmd_th )) vth = cmd_th;

                x += vx*cos(th)*sim_interval;
                y += vx*sin(th)*sim_interval;
                th += vth*sim_interval;

                py::list new_pt;
                new_pt.append(x);
                new_pt.append(y);
                new_pt.append(th);
                traj.append(new_pt);
            }

            //now we apply a stop
            b_ax = false;
            if (0 > vx) {
                acc_x = accs[0];
                b_ax = true;
            } else {
                acc_x = -1 * (double) accs[2];
            }

            b_ath = false;
            if (0 > vth) {
                acc_th = accs[1];
                b_ath = true;
            } else {
                acc_th = -1 * (double) accs[3];
            }

            for (double i=0; i<=time_decc; i+=sim_interval) {
                vx += acc_x*sim_interval;

                //check if we have already attained our velocity
                if (fabs(vx) < 1e-6) vx = 0;

                //check if we have exceeded our target
                if (b_ax && (vx > 0 )) vx = 0;
                if (!b_ax && (vx < 0 )) vx = 0;

                vth += acc_th*sim_interval;

                if (fabs(vth) < 1e-6) vth = 0;
                if (b_ath && (vth > 0 )) vth = 0;
                if (!b_ath && (vth < 0 )) vth = 0;

                x += vx*cos(th)*sim_interval;
                y += vx*sin(th)*sim_interval;
                th += vth*sim_interval;

                py::list new_pt;
                new_pt.append(x);
                new_pt.append(y);
                new_pt.append(th);
                traj.append(new_pt);


                bool stopped = (fabs(vth) < 1e-2) && (fabs(vx) < 1e-2);
                if (stopped) break;

            }

            return_val = traj;
        """
        traj = weave.inline(code, ['cmd_vel', 'current_speed', 'accs','sim_interval', 'delay_time',  'sim_interval_sq',
                                   'time_applied', 'max_acc_x', 'max_acc_th', 'max_decc_x', 'max_decc_th',
                                   'time_decc'], type_converters=converters.blitz)

        return traj

    def findLimitedDWACmd(self, cmd):
        """
        Performs obstacle avoidance with a DWA variant
        @param: cmd the desired command
        @returns: cmd the augmented command
        """
        #if no collision, we just return
        if self.checkForCollision(cmd, rough=True, publish=True) == False:
            return cmd

        #create a set of commands to test
        best_cmd = [0.0,0.0]
        cmds = [ [scale*cmd[0], scale*cmd[1]] for scale in np.arange(1.0, -0.2, -0.2) ]
        #print cmds
        for cmd_to_test in cmds:
            if not self.checkForCollision(cmd_to_test, rough=True):
                best_cmd = cmd_to_test
                break

        return best_cmd

    #=============================================================================================
    # VFH Functions
    #=============================================================================================

    def computePolarRangeHist(self):
        '''
        Generates a polar histogram given the obstacle map
        '''
        prh_resolution = self.prh_resolution
        max_considered_dist = self.max_considered_dist


        polar_range_hist_len = int((2.0*np.pi/prh_resolution))
        obstacles = list(self.obstacle_map.obstacles_in_memory)

        n_obstacles = len(obstacles)
        prh_smooth_window = self.prh_smooth_window
        pi = np.pi
        code = """
                int i;
                double *polar_range_hist = new double[polar_range_hist_len];
                double *temp_polar_range_hist = new double[polar_range_hist_len];

                for (int i=0; i<polar_range_hist_len; i++) {
                    polar_range_hist[i] = max_considered_dist;

                }

                for (i=0; i<n_obstacles; i++) {
                    double direction = atan2(obstacles[i][1], obstacles[i][0]) + pi;
                    double r = sqrt( pow(obstacles[i][1],2.0) + pow(obstacles[i][0],2.0));
                    if (r > max_considered_dist) r = max_considered_dist;

                    //int key = ((int) round(direction/prh_resolution))%polar_range_hist_len;
                    int key = ((int) round(direction/prh_resolution));

                    while (key < 0) {
                        key = polar_range_hist_len + key;
                    }
                    key = key%polar_range_hist_len;

                    if (polar_range_hist[key] > r) {
                        polar_range_hist[key] = r;
                    }
                }

                for (int i=0; i<polar_range_hist_len; i++) {
                    temp_polar_range_hist[i] = polar_range_hist[i];

                }

                for (i=0; i<polar_range_hist_len; i++) {
                    int lb = i-prh_smooth_window;
                    int up = i+prh_smooth_window+1;
                    double sum = 0.0;
                    for (int j=lb; j < up; j++) {

                        int key = j;
                        while (key < 0) key = polar_range_hist_len + key;

                        key = key%polar_range_hist_len;

                        sum += (1.0 - fabs(j-i)/(polar_range_hist_len + 1.0))*polar_range_hist[key];
                    }
                    temp_polar_range_hist[i] = sum/(double) (prh_smooth_window*2.0);
                }


                py::list result;
                for (i =0; i<polar_range_hist_len; i++) {
                    result.append(temp_polar_range_hist[i]);
                }

                delete [] polar_range_hist;
                delete [] temp_polar_range_hist;

                return_val = result;
            """
        polar_range_hist = weave.inline(code, ['prh_resolution', 'polar_range_hist_len',
                                               'obstacles', 'pi',
                                               'n_obstacles', 'prh_smooth_window', 'max_considered_dist'],
                                                type_converters=converters.blitz)

        self.polar_range_hist_lock.acquire()
        #self.polar_range_hist = 1.0 - np.exp(-1.0*np.array(polar_range_hist)/1.25)
        self.polar_range_hist = np.array(polar_range_hist)
        self.polar_range_hist_lock.release()

    def getAngleDiff_deg(self,a1, a2):
        dif = np.mod(np.abs(a1 - a2), 360)

        if dif > 180:
            dif = 360 - dif

        return dif


    def getClosenessMeasure(self, x, y, sd):
        return np.exp(-(np.linalg.norm(np.array(x) - np.array(y)))/sd)

    def findVFHCmd(self, cmd):
        """
        Performs obstacle avoidance with a VFH variant
        @param: cmd the desired command
        @returns: cmd the augmented command
        """
        self.computePolarRangeHist()
        self.publishPolarHistogram()

        if cmd == [0, 0]:
            return cmd

        #modify based on polar histogram
        #get new turning velocity
        prh_threshold = 0.01

        #setup variables we'll use
        local_prh = np.array(self.polar_range_hist)
        prh_resolution = self.prh_resolution
        prh_resolution_deg = self.prh_resolution*180/np.pi
        polar_range_hist_len = int((2.0*np.pi/prh_resolution))
        degree_angles = np.arange(-180, 180, prh_resolution_deg )

        assert(len(degree_angles) == polar_range_hist_len)
        closeness_scores = np.array([0.0]*polar_range_hist_len)

        #compute the closeness of the degree to the user's command
        for i in range(len(closeness_scores)):
            closeness_scores[i] = self.getAngleDiff_deg(cmd[1]*180/np.pi, degree_angles[i])
            closeness_scores[i] = np.exp(-np.fabs(closeness_scores[i])*np.pi/180) #convert to radians

        #compute the zone scores
        zone_scores = closeness_scores*self.closeness_weight + local_prh*self.free_zone_weight

        #publish the zone scores
        self.publishZoneScores(zone_scores, prh_resolution)

        #threshold the zone values
        for i in range(len(local_prh)):
            if local_prh[i] < prh_threshold:
                zone_scores[i] = -100

        #search for the best value
        range_to_search = len(zone_scores)/4
        midpoint = round(len(local_prh)/2)

        max_item = np.argmax(zone_scores[
                             (midpoint - range_to_search):(midpoint + range_to_search)])

        new_turning_vel = degree_angles[max_item + (midpoint-range_to_search)]

        #perform conversion
        new_turning_vel = self.turning_coeff*(new_turning_vel/180)


        #limit to a maximum
        new_turning_vel = np.sign(new_turning_vel)*max( abs(self.max_turn_vel), abs(new_turning_vel))


        #get new forward velocity - this is identify to basic 
        obstacles = list(self.obstacle_map.obstacles_in_memory)

        curr_xspeed = abs(self.curr_vel[0])
        if curr_xspeed > 0.25:
            #the faster you are going, the more modification is performed
            front_modifier = 0.5 + 0.5*(self.max_vel - curr_xspeed)
            side_modifier = 0.5 + 0.5*(self.max_vel - curr_xspeed)
        else:
            front_modifier = 1.0
            side_modifier = 1.0

        for obs in obstacles:
            new_modifier = 1.0
            if (np.sign(cmd[0])*obs[0] > 0) and (abs(obs[1]) < self.robot.footprint[1][1]):
                dist = abs(obs[0])

                if dist < 0.5:
                    new_modifier = 0.0
                elif dist > 2.0:
                    new_modifier = 1.0
                else:
                    new_modifier = (dist/2.0)

            front_modifier = min(new_modifier, front_modifier)
        new_forward_vel = front_modifier*cmd[0]

        #change the turning velocity if necessary (when going backwards)
        if new_forward_vel < 0:
            new_turning_vel *= -1

        #set the best command
        best_cmd = [new_forward_vel, new_turning_vel]

        return best_cmd


    #=============================================================================================
    # Basic Collision Prevention
    #=============================================================================================
    def findBasicSafeguardedCmd(self, cmd):
        """
        Performs basic "ad-hoc" safeguarding
        @param: cmd the desired command
        @returns: cmd the augmented command
        """
        obstacles = list(self.obstacle_map.obstacles_in_memory)

        curr_xspeed = abs(self.curr_vel[0])
        if curr_xspeed > 0.25:
            #the faster you are going, the more modification is performed
            front_modifier = 0.5 + 0.5*(self.max_vel - curr_xspeed)
            side_modifier = 0.5 + 0.5*(self.max_vel - curr_xspeed)
        else:
            front_modifier = 1.0
            side_modifier = 1.0

        for obs in obstacles:
            new_modifier = 1.0
            if (np.sign(cmd[0])*obs[0] > 0) and (abs(obs[1]) < self.robot.footprint[1][1]):
                dist = abs(obs[0])

                if dist < 0.7:
                    new_modifier = 0.0
                elif dist > 2.0:
                    new_modifier = 1.0
                else:
                    new_modifier = (dist/2.0)

            front_modifier = min(new_modifier, front_modifier)

        for obs in obstacles:
            new_modifier = 1.0
            if (np.sign(cmd[1])*obs[1] > 0) and (abs(obs[0]) < self.robot.footprint[2][0]):
                dist = abs(obs[1])

                if dist < 0.50:
                    new_modifier = 0.0
                elif dist > 2.0:
                    new_modifier = 1.0
                else:
                    new_modifier = (dist/2.0)

            side_modifier = min(new_modifier, side_modifier)
        rospy.loginfo('Basic Modifiers: ' + str(front_modifier) + ', ' + str(side_modifier))

        best_cmd = [front_modifier*cmd[0], side_modifier*cmd[1]]
        return best_cmd


    #=============================================================================================
    # ROS Publishing Functions
    #=============================================================================================

    def publishCmd(self, cmd):
        """
        Publishes the velocity command
        """
        cmd_to_publish = Twist()
        cmd_to_publish.linear.x = cmd[0]
        cmd_to_publish.angular.z = cmd[1]
        self.cmd_pub.publish(cmd_to_publish)


    def publishObstacles(self):
        """
        Publishes the obstacles as markers
        """
        mk = Marker()
        mk.header.stamp = rospy.get_rostime()
        mk.header.frame_id = '/base_link'

        mk.ns='basic_shapes'
        mk.id = 0
        mk.type = Marker.POINTS
        mk.scale.x = 0.3
        mk.scale.y = 0.3
        mk.scale.z = 0.3
        mk.color.r = 1.0
        mk.color.a = 1.0

        for value in self.obstacle_map.obstacles_in_memory:
            p = Point()
            p.x = value[0]
            p.y = value[1]
            mk.points.append(p)


        self.obs_pub.publish(mk)


    def publishPolarHistogram(self):
        """
        Publishes the polar histogram
        """
        pc = LaserScan()
        pc.header.frame_id = "/base_link"
        pc.header.stamp = rospy.get_rostime()

        pc.angle_min = -np.pi
        pc.angle_max = np.pi
        pc.angle_increment = self.prh_resolution
        pc.range_min = 0.00
        pc.range_max = 5.0

        self.polar_range_hist_lock.acquire()
        for r in self.polar_range_hist:
            pc.ranges.append(r)

        self.polar_range_hist_lock.release()
        self.polar_hist_pub.publish(pc)


    def publishZoneScores(self,data, prh_resolution):
        """
        Publishes the zone scores
        """
        pc = LaserScan()
        pc.header.frame_id = "/base_link"
        pc.header.stamp = rospy.get_rostime()

        pc.angle_min = -np.pi
        pc.angle_max = np.pi
        pc.angle_increment = prh_resolution
        pc.range_min = 0.00
        pc.range_max = 5.0

        for r in data:
            pc.ranges.append(r)

        self.zone_score_pub.publish(pc)

    def publishProjection(self, data):
        """
        Publishes the forward projection/simulation
        """
        proj = PoseArray()
        proj.header.stamp = rospy.get_rostime()
        proj.header.frame_id = "/base_link"
        for pt in data:
            pos = Pose()
            pos.position.x = pt[0]
            pos.position.y = pt[1]
            orient = quaternion_from_euler(0,0,pt[2])
            pos.orientation.x = orient[0]
            pos.orientation.y = orient[1]
            pos.orientation.z = orient[2]
            pos.orientation.w = orient[3]
            proj.poses.append(pos)

        self.projection_pub.publish(proj)

    def publishTimeTaken(self, data):
        """
        Publishes the time taken
        """
        time_taken = Float32()
        time_taken.data = data
        self.time_taken_pub.publish(data)

    #=============================================================================================
    # Main Functions
    #=============================================================================================

    def updateAndPublish(self):

        #first we update the obstacle map
        start_time = time.time()

        rospy.loginfo("Updating Obstacle map")
        self.updateObstacleMap()

        rospy.loginfo("Publishing Obstacles")
        self.publishObstacles()

        rospy.loginfo("Finding Shared-Control Command")
        start_time = rospy.get_time()
        curr_cmd = self.curr_cmd

        #uncomment to choose the algorithm you want to test
        best_cmd = self.findBasicSafeguardedCmd(curr_cmd)
        #best_cmd = self.findLimitedDWACmd(curr_cmd)
        #best_cmd = self.findVFHCmd(curr_cmd)

        elapsed_time = rospy.get_time() - start_time
        self.publishTimeTaken(elapsed_time)

        rospy.loginfo("Publishing Shared-Control Command")
        self.publishCmd(best_cmd)

        return

    def startLoop(self, rate=10):
        rospy.loginfo("Starting shared control")
        r = rospy.Rate(rate)
        try:
            while not rospy.is_shutdown():
                self.updateAndPublish()
                r.sleep()
        except rospy.ROSInterruptException:
            pass

        return


if __name__ == "__main__":


    #set up the p3at footprint
    p3at_footprint = [ (-0.35, -0.3),
        (-0.35, 0.3),
        (0.35, 0.3),
        (0.35, -0.3)]

    #set up the laser position
    p3at_laser_position = [0.18, 0.0]

    #set up the p3at footprint
    real_p3at_footprint = [ (-0.35, -0.3),
        (-0.35, 0.3),
        (0.35, 0.3),
        (0.35, -0.3)]

    #set up the laser position
    real_p3at_laser_position = [0.18, 0.0]


    #create the robot profile
    rospy.loginfo("Creating Robot Profile")
    real_p3at = RobotProfile(footprint=real_p3at_footprint,
        laser_position = real_p3at_laser_position,
        max_velocities=[2.0, 0.8],
        min_velocities=[-2.0, -0.8],
        max_accelerations=[0.3, 1.0],
        max_decelerations=[0.3, 1.0],
        acceleration_params=[0.3, 1.0],
        deceleration_params=[0.3, 1.0])

    p3at = RobotProfile(footprint=real_p3at_footprint,
        laser_position = real_p3at_laser_position,
        max_velocities=[2.0, 0.8],
        min_velocities=[-2.0, -0.8],
        max_accelerations=[1.0, 1.0],
        max_decelerations=[1.0, 1.0],
        acceleration_params=[1.0, 1.0],
        deceleration_params=[1.0, 1.0])


    #create the Shared Control object and start
    rospy.loginfo("Creating Shared Control Object")
    sc = SharedControl(p3at)

    sc.startLoop(rate = 20)



