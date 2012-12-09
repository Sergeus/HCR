#!/usr/bin/env python
import roslib; roslib.load_manifest('printerListenerPy')
import rospy
from messages.msg import printReceipt
import socket
import sys
import time
import subprocess

def callback(data):
    HOST, PORT = "localhost", 9999

    # SOCK_DGRAM is the socket type to use for UDP sockets
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    sock.sendto("PRINT-REQUEST;Time=" + str(time.gmtime()), (HOST, PORT))


def listener():
    rospy.init_node('printerListenerPy', anonymous=True)
    rospy.Subscriber("printerListenerPy", printReceipt , callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
