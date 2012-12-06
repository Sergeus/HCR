#!/usr/bin/env python

"""
recognizer.py is a wrapper for pocketsphinx.
  parameters:
    ~lm - filename of language model
    ~dict - filename of dictionary
  publications:
    ~output (std_msgs/String) - text output
  services:
    ~start (std_srvs/Empty) - start speech recognition
    ~stop (std_srvs/Empty) - stop speech recognition
"""

import roslib; roslib.load_manifest('pocketsphinx')
import rospy

import pygtk
pygtk.require('2.0')
import gtk

import gobject
import pygst
pygst.require('0.10')
gobject.threads_init()
import gst

from std_msgs.msg import String
from std_srvs.srv import *
#from pocketsphinx.srv import string

class recognizer(object):
    """ GStreamer based speech recognizer. """

    base_dir = "../../../knowledge_base/"

    def __init__(self):
        """ Initialize the speech pipeline components. """
        rospy.init_node('recognizer')
        self.pub = rospy.Publisher('ps_out',String)
        rospy.on_shutdown(self.shutdown)

        # services to start/stop recognition
        rospy.Service("ps_start", Empty, self.start)
        rospy.Service("ps_stop", Empty, self.stop)
        #rospy.Service("ps_change_lm", string, self.change_model)

        # configure pipeline
        self.pipeline = gst.parse_launch('gconfaudiosrc ! audioconvert ! audioresample '
                                         + '! vader name=vad auto-threshold=true '
                                         + '! pocketsphinx name=asr ! fakesink')
        asr = self.pipeline.get_by_name('asr')
        asr.connect('partial_result', self.asr_partial_result)
        asr.connect('result', self.asr_result)
        asr.set_property('configured', True)
        asr.set_property('dsratio', 1)

        # parameters for lm and dic
        #try:
        #    lm_ = rospy.get_param('~lm')
        #    print lm_
        #except:
        #    rospy.logerr('Please specify a language model file')
        #    return
        #try:
        #    dict_ = rospy.get_param('~dict')
        #    print dict_
        #except:
        #    rospy.logerr('Please specify a dictionary')
        #    return

        temp_lm = self.base_dir + "complete.lm"
        temp_dic = self.base_dir + "complete.dic"
        asr.set_property('lm', temp_lm)
        asr.set_property('dict',temp_dic)

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect('message::application', self.application_message)
        self.start(None)
        gtk.main()

    def shutdown(self):
        """ Shutdown the GTK thread. """
        gtk.main_quit()

    def start(self, msg):
        self.pipeline.set_state(gst.STATE_PLAYING)
        return EmptyResponse()

    def stop(self):
        self.pipeline.set_state(gst.STATE_PAUSED)
        #vader = self.pipeline.get_by_name('vad')
        #vader.set_property('silent', True)
        return EmptyResponse()

    #def change_model(self, name):
    #    self.stop()
    #    temp_lm = self.base_dir + name.msg + ".lm"
    #    temp_dic = self.base_dir + name.msg + ".dic"
    #    asr = self.pipeline.get_by_name('asr')
    #    asr.set_property('lm', temp_lm)
    #    asr.set_property('dict',temp_dic)
    #    self.start(None)

    def asr_partial_result(self, asr, text, uttid):
        """ Forward partial result signals on the bus to the main thread. """
        struct = gst.Structure('partial_result')
        struct.set_value('hyp', text)
        struct.set_value('uttid', uttid)
        asr.post_message(gst.message_new_application(asr, struct))

    def asr_result(self, asr, text, uttid):
        """ Forward result signals on the bus to the main thread. """
        struct = gst.Structure('result')
        struct.set_value('hyp', text)
        struct.set_value('uttid', uttid)
        asr.post_message(gst.message_new_application(asr, struct))

    def application_message(self, bus, msg):
        """ Receive application messages from the bus. """
        msgtype = msg.structure.get_name()
        if msgtype == 'partial_result':
            self.partial_result(msg.structure['hyp'], msg.structure['uttid'])
        if msgtype == 'result':
            self.final_result(msg.structure['hyp'], msg.structure['uttid'])

    def partial_result(self, hyp, uttid):
        """ Delete any previous selection, insert text and select it. """
        print "Partial: " + hyp

    def final_result(self, hyp, uttid):
        """ Insert the final result. """
        msg = String()
        msg.data = str(hyp.lower())
        rospy.loginfo(msg.data)
        self.pub.publish(msg)

if __name__=="__main__":
    r = recognizer()

