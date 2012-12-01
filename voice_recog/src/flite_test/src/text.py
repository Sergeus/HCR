#!/usr/bin/env python
import roslib; roslib.load_manifest('flite_test')
import rospy
from std_msgs.msg import String
from subprocess import call


def callback(data):
    rospy.loginfo(rospy.get_name() + ": I heard %s" % data.data)
    call(["flite", "-t", data.data])


def listener():
    rospy.init_node('text', anonymous=True)
    rospy.Subscriber("TTS", String, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
