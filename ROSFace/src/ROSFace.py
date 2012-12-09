#!/usr/bin/env python
import roslib; roslib.load_manifest('ROSFace')
import rospy
from std_msgs.msg import String
from messages.msg import faceRequests
from messages.msg import HeadCoordinates
import subprocess
import os

process = None

def emotionCallback(data):
    rospy.loginfo("Changing emotion/talking status. emotion=" + str(data.emotion) + "talking" + str(data.talking))
    
    global process

    if (data.talking == 1):
        process.stdin.write("e=" + data.emotion + ";t=true")
    else:
        process.stdin.write("e=" + data.emotion + ";t=false")

def headCallback(data):
    rospy.loginfo("Changing eye coordinates. ed=" + str(data.x) + ":" + str(data.y) + ":" + str(data.z))

    global process
    process.stdin.write("ed=" + str(data.x) + ":" + str(data.y) + ":" + str(data.z))

def listener():
    rospy.init_node('ROSFace', anonymous=True)

    rospy.Subscriber("faceRequests", faceRequests, emotionCallback)
    rospy.Subscriber("kinect_heads", HeadCoordinates, headCallback)

    global process

    directory = os.environ['ROS_DIR'] + "RobotFace/node/app.js"
    rospy.loginfo("starting " + directory)
    process = subprocess.Popen(["nodemon", directory], stdin=subprocess.PIPE)
    rospy.spin()

if __name__ == '__main__':
    listener()
