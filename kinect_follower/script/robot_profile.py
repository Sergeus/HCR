#!/usr/bin/env python
#numpy and scipy imports
import numpy as np



#Robot Profile class.

class RobotProfile():
    def __init__(self, footprint,
                 laser_position,
                 max_velocities,
                 min_velocities,
                 acceleration_params,
                 deceleration_params,
                 max_accelerations,
                 max_decelerations):

        #body frame (a list of points specified in the counter clockwise direction
        #from the bottom. (0,0) is the center of the robot.
        self.footprint = footprint

        #set the laser position
        self.laser_pos = laser_position

        #computer the outer and inner radius of the robot
        #if your robot is purely symmetrical, then they are equal.
        self.inner_radius = footprint[0][0]
        self.outer_radius = footprint[0][0]
        for pt in self.footprint:
            for r in pt:
                self.outer_radius = max(np.fabs(r), self.outer_radius)
                self.inner_radius = min(np.fabs(r), self.inner_radius)


        #velocities
        self.max_vx = max_velocities[0]
        self.max_vth = max_velocities[1]

        self.min_vx = min_velocities[0]
        self.min_vth = min_velocities[1]

        #accelerations
        self.acc_x = acceleration_params[0]
        self.acc_th = acceleration_params[1]

        #decelerations
        self.dacc_x = deceleration_params[0]
        self.dacc_th = deceleration_params[1]

        #max accelerations
        self.max_acc_x = max_accelerations[0]
        self.max_acc_th = max_accelerations[1]

        #max decelerrations
        self.max_dacc_x = max_decelerations[0]
        self.max_dacc_th = max_decelerations[1]

        return


