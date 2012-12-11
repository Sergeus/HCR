#!/usr/bin/env python
import roslib; roslib.load_manifest('printerListenerPy')
import rospy
from messages.msg import printReceipt
import socket
import sys
import time
import subprocess
import os

def callback(data):
    HOST, PORT = "192.168.0.5", 9999
    rospy.loginfo("Print request received. Sendinf UDP to " + HOST + ":" + str(PORT))
    # SOCK_DGRAM is the socket type to use for UDP sockets
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    timeNow = str(time.mktime(time.gmtime())).split(".")
    sock.sendto(timeNow[0], (HOST, PORT))
    echoStr = "echo " + str(data.mode) + ", " + timeNow[0] + " >> " + os.environ['ROS_DIR'] + "charleslog"
    #subprocess.call(["echo", str(timeNow[0]), ">>", "/home/chris/charleslog"])
    subprocess.call(echoStr, shell=True)

def listener():
    rospy.init_node('printerListenerPy', anonymous=True)
    rospy.Subscriber("printerListenerPy", printReceipt , callback)
    rospy.spin()


if __name__ == '__main__':
   listener()
