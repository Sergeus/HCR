#!/usr/bin/env python
import roslib; roslib.load_manifest('ROSFace')
import rospy
from std_msgs.msg import String
from messages.msg import faceRequests
import subprocess

process = None

def callback(data):
    rospy.loginfo("Chaning emotion/talking status")
    
    global process

    if (data.talking == 1):
        process.stdin.write("e=" + data.emotion + ";t=true")
    else:
        process.stdin.write("e=" + data.emotion + ";t=false")

def listener():
    rospy.init_node('ROSFace', anonymous=True)

    rospy.Subscriber("faceRequests", faceRequests, callback)

    global process
    process = subprocess.Popen(["nodemon", "/home/chris/HCR-Repo/HCR/RobotFace/node/app.js"], stdin=subprocess.PIPE)
    rospy.spin()

if __name__ == '__main__':
    listener()
