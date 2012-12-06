#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_speech_engine')
import rospy
from std_msgs.msg import String
#from ros_speech_engine.srv import string
import time
import random
import os
from subprocess import call

class Utterance:

    text = "NULL"

    def __init__(self, text):
        self.text = text

    def containsName(self):
        return self.extractWord('names.txt') != "NULL"

    def containsLocation(self):
        return self.extractWord('locations.txt') != "NULL"

    def containsYes(self):
        if self.extractWord('yes.txt') != "NULL":
            return True
        else :
            return False
        

    def containsNo(self):
        if self.extractWord('no.txt') != "NULL":
            return True
        else :
            return False
            
    def getName(self):
        return self.extractWord('names.txt')
    
    def getLocation(self):
        return self.extractWord('locations.txt')
        
    def extractWord(self, fname):
        words = self.text.split()
        temp =  os.environ['ROS_VOICE'] + "ros_speech_engine/src/" + fname

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
        call(["flite", "-t", sentence])


# Main functional loop
if __name__ == '__main__':

    ps = PocketSphinx()
    ss = SpeechSynthesis()

    rospy.init_node('ros_speech_engine', anonymous=True)
    
    # If the service is started
    if True:

        ps.start()
        ss.start()

        state = "ASK_NAME"
        name = "NULL"
        
        random.seed()
        
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
                    ss.speak("Hello " + name + ".  My name is CHARLES.")
                else :    
                    ss.speak("Hello.  My name is CHARLES")
                
                state = "CHOOSE_STATE"
                
            elif state == "CHOOSE_STATE":
                   
                randomNum = random.randint(0, 2)
                if randomNum == 0:
                    state = "ASK_LOCATION"
                elif randomNum == 1:
                    state = "ASK_CAKE"
                else:
                    state = "ASK_MEETING"
                
            elif state == "ASK_LOCATION":
                
                ss.speak("Where are you going to?")
                location = "NULL"
                state = "RECOG_LOCATION"
            
            elif state == "RECOG_LOCATION":
            
                location = Utterance(ps.listen()).getLocation()
                if ("imperial" in location) or (location =="school") or (location =="lectures") or (location =="university") or (location =="college") :
                    ss.speak("I can teach you everything there is to know.  A to Z. From Android to Robot.")
                elif (location == "underground") or (location == "tube") or (location == "station")  or (location == "line"):
                    ss.speak("It is cold and dark and emotionless down there.  Not like me of course")
                elif (location  == "history museum") or (location == "history") or (location == "V and A") :
                    ss.speak("That is all about the past.  Concern yourself with the future.")
                elif (location  == "science") :
                    ss.speak("Ah my home.  I have many friends there")
                else :
                    ss.speak("That sounds so very very exciting.  However, I can not travel up stairs.")
                state = "ASK_INTERESTED"
            
            elif state == "ASK_CAKE":
                ss.speak("Do you like cake?")
                cake = "NULL"
                state = "RECOG_CAKE"
                attempt = 0
                
            elif state == "RECOG_CAKE":  
                cake = Utterance(ps.listen())
                state = "ASK_INTERESTED"
                
                if cake.containsYes() == True:
                    ss.speak("Unlucky. All the cake is gone.")
                elif cake.containsNo() == True:
                    ss.speak("It must be wasted on you. I dream of cake.  And electric sheep")
                else:
                    attempt = attempt + 1
                    if attempt < 2 :
                        ss.speak("I am sorry.  Could you say that again?")
                        
                        state = "RECOG_CAKE" 
                    else:
                        ss.speak("How unfortunate.  Perhaps you are wiser than you first seemed.")
                        state = "ASK_INTERESTED"        
                
            elif state == "ASK_MEETING":
                
                ss.speak("Have you ever met a robot before?")
                
                meeting = "NULL"
                state = "RECOG_MEETING"
            
            elif state == "RECOG_MEETING":
            
                meeting = Utterance(ps.listen())
                
                if meeting.containsYes() == True:
                    ss.speak("I think we might become best of friends sooner than I thought...")
                elif meeting.containsNo() == True:
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

                if response.containsYes()  == True:
                    print "SUCCESS: Ticket printed"
                elif response.containsNo() == True:
                    print "UNLUCKY: Ticket not printed"
                else:
                    print "SUCCESS: Ticket printed"
                
                ss.speak("It has been nice speaking to you.")
                state = "ASK_NAME"
            elif state == "ERROR":
                print "Error state"
                break
        
        ps.stop()
        ss.stop()

