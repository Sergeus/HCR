#!/usr/bin/env python

#the following three lines should be familiar by now. 
import roslib
roslib.load_manifest('kinect_follower')

# Load (import) rospy
import rospy
from numpy import sin, cos
# Load sonarArray message from p2os
from p2os_driver.msg import SonarArray

# Load PointCloud from sensor_msgs
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from sensor_msgs.msg import ChannelFloat32
from std_msgs.msg import Float32


class SonarToPointCloudConverter:

    #Sonar positions
    #(hardcoded, but maybe they should be determined at runtime)
    sonar_positions = [
        +1.570796326794897, 
        +0.872664625997165, 
        +0.523598775598299,
        +0.174532925199433,
        -0.174532925199433,
        -0.523598775598299,
        -0.872664625997165,
        -1.570796326794897,
        -1.570796326794897,
        -2.268928027592628,
        -2.617993877991494,
        -2.967059728390360,
        +2.967059728390360,
        +2.617993877991494,
        +2.268928027592628,
        +1.570796326794897
    ]

    # the CLASS initialization code
    def __init__(self, x_scale = 1.0, y_scale = 1.0):
    
        
        # initialize the node with the name 'joystick_to_cmd_vel'
        rospy.init_node('sonar_to_pointcloud')
        
        # set up a publisher to topic 'cmd_vel' publishing Twist messages
        self.pub = rospy.Publisher('sonar_pc', PointCloud)
        
        # The following line creates a subcriber to the joystick topic
        # it has 3 arguments:
        #  1. the topic to subscribe to
        #  2. the message type
        #  3. the "callback" function to run when a message is received
        rospy.Subscriber('sonar', SonarArray, self.sonarCallback)
        
    # this is the callback function 
    # it is run whenever a new SonarArray message is received
    def sonarCallback(self, data):
        
        sonarpc = PointCloud()
        sonarpc.header.stamp = data.header.stamp
        sonarpc.header.frame_id = 'base_link'
        sonarpc.channels = [ChannelFloat32()]
        sonarpc.channels[0].values = []

        counter = 0
        for r in data.ranges:
            #transform the pt
            tpc = Point32()
            r = r + 0.24
            tpc.x = cos(self.sonar_positions[counter])*r
            tpc.y = sin(self.sonar_positions[counter])*r
            tpc.z = 0.23
            counter += 1
            #add this to the cloud
            sonarpc.points.append(tpc)
            sonarpc.channels[0].values.append(100)

        # Publish the new cmd message using publish
        self.pub.publish(sonarpc)
        
        
    # call this function to start the listener 
    def startListener(self):
        # the rospy.spin() member function starts the callbacks
        rospy.spin()
        

# our main function
if __name__ == '__main__':
    try:
        # instantiate the class and start the listener
        jsconv = SonarToPointCloudConverter()
        jsconv.startListener()
    except rospy.ROSInterruptException: 
        pass
        
        
