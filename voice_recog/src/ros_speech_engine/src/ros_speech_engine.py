#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_speech_engine')
import rospy
from std_msgs.msg import String
#from ros_speech_engine.srv import string
import time
import random

class Utterance:

    text = "NULL"

    def __init__(self, text):
        self.text = text

    def containsName(self):
        return self.extractWord('names.txt') != "NULL"

    def containsLocation(self):
        return self.extractWord('locations.txt') != "NULL"

    def containsYes(self):
      #  print self.extractWord('yes.txt')
        if self.extractWord('yes.txt') == "NULL" :
            return False
        else:
            return True

    def containsNo(self):
      #  print self.extractWord('no.txt')
        if self.extractWord('no.txt') == "NULL" :
            return False
        else:
            return True
            
    def containsYesNo(self):
        words = self.text.split()
        temp = "/home/chris/ros_workspace/sandbox/ros_speech_engine/src/"

        for word in words:
            if word in open(temp + 'yes.txt').read():
                return "Yes"
        for word in words:
            if word in open(temp + 'no.txt').read():
                return "No"
                
        return "NULL"
        
    def getName(self):
        return self.extractWord('names.txt')
    
    def getLocation(self):
        return self.extractWord('location.txt')
        
    def extractWord(self, fname):
        words = self.text.split()
        temp = "/home/chris/ros_workspace/sandbox/ros_speech_engine/src/" + fname

        for word in words:
            if word in open(temp).read():
                return word
    
        return "NULL"

class PocketSphinx:

    text = "NULL"

    def start(self):
        # Will eventually start recogniser
        print "Started PocketSphinx node"
        foo = self.callback
        rospy.Subscriber("ps_out", String, foo)
        return True

    def stop(self):
        # Will eventually stop recogniser
        print "Stopped PocketSphinx node"
        return True

    def listen(self):
        # Will eventually listen to mic
        # return raw_input('Input: ')
        self.text = "NULL"

        while self.text == "NULL":
            time.sleep(1)

        return self.text

    #def change_model(self, name):
    #    rospy.wait_for_service('ps_change_lm')
    #    try:
    #        temp = rospy.ServiceProxy('ps_change_lm', string)
    #        temp(name)
    #    except rospy.ServiceException, e:
    #        print "Service call failed: %s"%e

    def callback(self, data):
        print data.data
        self.text = data.data

class SpeechSynthesis:

    pub = rospy.Publisher('TTS', String)

    def __init__(self, topic):
        self.pub = rospy.Publisher(topic, String)
        rospy.init_node('ros_speech_engine', anonymous=True)

    def start(self):
        # Will eventually start TTS
        print "Started SpeechSynthesis node"
        return True

    def stop(self):
        # Will eventually stop TTS
        print "Stopped SpeechSynthesis node"
        return True

    def speak(self, sentence):
        ## Sends text to TTS
        rospy.loginfo(sentence)
        self.pub.publish(String(sentence))
        # print sentence
        # return True

# Main functional loop
if __name__ == '__main__':

    ps = PocketSphinx()
    ss = SpeechSynthesis('TTS')

    # If the service is started
    if True:

        ps.start()
        ss.start()

        state = "ASK_NAME"
        name = "NULL"

        # While we have user's attention
        while True:

            if state == "ASK_NAME":

                #ps.change_model("names")
                ss.speak("Hello, what is your name?")
                name = "NULL"
                state = "RECOG_NAME"

            elif state == "RECOG_NAME":
               
                name = Utterance(ps.listen()).getName()
                
                if name != "NULL" :
                    state = "HELLO_NAME"
                else:
                    state = "ERROR"
                
            elif state == "HELLO_NAME":
            
                ss.speak("Hello " + name)
                
                randomNum = random.randint(0, 1)
                if randomNum == 0:
                    state = "ASK_LOCATION"
                else:
                    state = "ASK_MEETING"
              
            elif state == "ASK_LOCATION":
                
                ss.speak("Where are you going to?")
                location = "NULL"
                state = "RECOG_LOCATION"
            
            elif state == "RECOG_LOCATION":
            
                location = Utterance(ps.listen())
 #               if location == (
 #                 case
                state = "ASK_INTERESTED"
            
            elif state == "ASK_MEETING":
                
                ss.speak("Have you ever met a robot before?")
                
                meeting = "NULL"
                state = "RECOG_MEETING"
            
            elif state == "RECOG_MEETING":
            
                meeting = Utterance(ps.listen())
                
                if meeting.containsYesNo() == Yes:
                    ss.speak("I think we might become best of friends sooner than I thought...")
                elif meeting.containsYesNo() == No:
                    ss.speak("That is most unfortunate.  You have missed out...")
                else:
                    ss.speak("Your words confuse me.")
                state = "ASK_INTERESTED"        
                            
            elif state == "ASK_INTERESTED":

                #ps.change_model("yesno")
                ss.speak("Would you be interested in finding out more about this experiment?")
                state = "RECOG_INTERESTED"

            elif state == "RECOG_INTERESTED":

                response = Utterance(ps.listen())
               
                if response.containsYesNo() == "Yes":
                    print "SUCCESS: Ticket printed"
                    state = "ASK_NAME"
                elif response.containsYesNo() == "No":
                    print "SUCCESS: Ticket not printed"
                    state = "ASK_NAME"
                else:
                    state = "ERROR"

            elif state == "ERROR":
                print "Error state"
                break
        
        ps.stop()
        ss.stop()

