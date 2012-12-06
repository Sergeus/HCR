#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_speech_engine')
import rospy
from std_msgs.msg import String
#from ros_speech_engine.srv import string
import time
import random
import os
from subprocess import call
from messages.msg import startstop
from messages.msg import printRequest #voicePrintRequests

class ROSControl:

    status = "STOP"

    def __init__(self):
        print "Started Pocket"
        foo = self.callback
        rospy.Subscriber("voice_regogSS", startstop, foo)

    def checkStatus(self):
        return self.status
        #STOP STARTSPEAKING STARTCONVERSING

    def callback(self, data):
        self.status = data.operation 


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
        self.text = "NULL"

        while self.text == "NULL":
            time.sleep(1)

        return self.text

    def callback(self, data):
        print data.data
        self.text = data.data


def speak(sentence):
    call(["flite", "-t", sentence])

def conversationStateMachine(ps, ros):

    # If the service is started
    if True:

        ps.start()

        state = "ASK_NAME"
        name = "NULL"
        
        random.seed()
        iterator = 0 # iterates through different parts of a conversation
        
        # While we have user's attention
        while (ros.checkStatus() != "STOP"):
        
            if state == "ASK_NAME":

                speak("Hello, what is your name?")
                name = "NULL"
                state = "RECOG_NAME"

            elif state == "RECOG_NAME":
               
                name = Utterance(ps.listen()).getName()
                
                if name != "NULL" :
                    speak("Hello " + name + ".  My name is CHARLES.")
                else :    
                    speak("Hello.  My name is CHARLES")
                
                state = "CHOOSE_STATE"
                
            elif state == "CHOOSE_STATE":
                   

                if iterator == 0:
                    state = "ASK_LOCATION"
                elif iterator == 1:
                    state = "ASK_CAKE"
                else:
                    state = "ASK_MEETING"
                iterator = (iterator + 1) % 3

            elif state == "ASK_LOCATION":
                
                speak("Where are you going to?")
                location = "NULL"
                state = "RECOG_LOCATION"
            
            elif state == "RECOG_LOCATION":
            
                location = Utterance(ps.listen()).getLocation()
                if ("imperial" in location) or ("college" in location) or ("school" in location) or ("lectures" in location) or ("university" in location) :
                    speak("I can teach you everything there is to know.  A to Z. From Android to Robot.")
                elif ("underground" in location) or ("tube" in location) or ("station" in location)  or ("line" in location):
                    speak("It is cold and dark and emotionless down there.  Not like me of course")
                elif ("science" in location) or ("robot" in location):
                    speak("Ah my home.  I have many friends there")
                elif ("history" in location) or ("V and A" in location) or ("museum" in location) or ("victoria" in location):
                    speak("That is all about the past.  Concern yourself with the future.")
                elif ("house" in location) or ("home" in location) or ("halls" in location):
                    speak("Home is where the heart is.  If I only had heart.")
                else :
                    speak("That sounds so very very exciting.  However, I can not travel up stairs.")
                state = "ASK_INTERESTED"
            
            elif state == "ASK_CAKE":
                speak("Do you like cake?")
                cake = "NULL"
                state = "RECOG_CAKE"
                attempt = 0
                
            elif state == "RECOG_CAKE":  
                cake = Utterance(ps.listen())
                state = "ASK_INTERESTED"
                
                if cake.containsYes() == True:
                    speak("Unlucky. All the cake is gone.")
                elif cake.containsNo() == True:
                    speak("It must be wasted on you. I dream of cake.  And electric sheep")
                else:
                    attempt = attempt + 1
                    if attempt < 2 :
                        speak("I am sorry.  Could you say that again?")
                        state = "RECOG_CAKE" 
                    else:
                        speak("How unfortunate.  Perhaps you are wiser than you first seemed.")    
                
            elif state == "ASK_MEETING":
                
                speak("Have you ever met a robot before?")
                
                meeting = "NULL"
                state = "RECOG_MEETING"
                attempt = 0
                
            elif state == "RECOG_MEETING":
            
                meeting = Utterance(ps.listen())
                state = "ASK_INTERESTED"   
                if meeting.containsYes() == True:
                    speak("I think we might become best of friends sooner than I thought...")
                elif meeting.containsNo() == True:
                    speak("That is most unfortunate.  You have missed out...")
                else:
                    attempt = attempt + 1
                    if attempt < 2 :
                        speak("I am sorry.  Could you say that again?")
                        state = "RECOG_MEETING"
                    else:
                        speak("Your words confuse me.")
                             
                            
            elif state == "ASK_INTERESTED":

                speak("Would you be interested in finding out more about this experiment?")
                state = "RECOG_INTERESTED"

            elif state == "RECOG_INTERESTED":

                response = Utterance(ps.listen())

                if response.containsYes()  == True:
                    print "SUCCESS: Ticket printed"
                elif response.containsNo() == True:
                    print "UNLUCKY: Ticket not printed"
                else:
                    print "SUCCESS: Ticket printed"
                
                speak("It has been nice speaking to you.")
                state = "ASK_NAME"
            elif state == "ERROR":
                print "Error state"
                break
        
        ps.stop()



# Main functional loop
if __name__ == '__main__':

    ps = PocketSphinx()
    ros = ROSControl()

    rospy.init_node('ros_speech_engine', anonymous=True)
    
    while True:

        if (ros.checkStatus() == "STARTSPEAKING"):
            #Say something
            print "SPEAK STUFF LOL"
        elif (ros.checkStatus() == "STARTCONVERSING"):
            conversationStateMachine(ps, ros)
        else:
            time.sleep(1)



