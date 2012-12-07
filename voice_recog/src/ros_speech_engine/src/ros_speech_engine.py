#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_speech_engine')
import rospy
import time
import random
import os
from std_msgs.msg import String
from subprocess import call
from messages.msg import startstop
from messages.msg import printRequest

class ROSControl:

    def __init__(self):
        self.status = "STOP"
        rospy.Subscriber("voice_regogSS", startstop, self.callback)

    def checkStatus(self):
        # returns "STOP", "STARTSPEAKING", or "STARTCONVERSING"
        return self.status

    def resetStatus():
        self.status = "STOP"

    def callback(self, data):
        self.status = data.operation 

class Printer:

    def __init__(self):
        self.pub = rospy.Publisher('voicePrintRequests', printRequest)

    def requestPrint(self):
        self.pub.publish()

class Utterance:

    def __init__(self, text):
        self.text = text

    def containsYes(self):
        if self.extractWord('yes.txt') != None:
            return True
        else :
            return False

    def containsNo(self):
        if self.extractWord('no.txt') != None:
            return True
        else :
            return False
            
    def getName(self):
        return self.extractWord('names.txt')
    
    def getLocation(self):
        return self.extractWord('locations.txt')
        
    def extractWord(self, fname):
        temp =  os.environ['ROS_VOICE'] + "ros_speech_engine/src/" + fname

        words = self.text.split()

        lines = open(temp).readlines()

        for line in lines:

            lineWords = line.split()

            for lineWord in lineWords:

                for word in words:

                    if word == lineWord:

                        print "[INFO] Found " + word
                        return word

        return None 

class PocketSphinx:

    def __init__(self):
        self.text = None
        rospy.Subscriber("ps_out", String, self.callback)

    def listen(self):
        self.text = None

        while self.text == None:
            time.sleep(1)

        return self.text

    def callback(self, data):
        print data.data
        self.text = data.data

def speak(sentence):
    call(["flite", "-t", sentence])

def retry(attempt, success_state, fail_state, fail_text):
    if attempt < 2 :
        speak("I am sorry.  Could you say that again?")
        return fail_state
    else:
        speak(fail_text) 
        return success_state 

def conversationStateMachine(ps, ros):
   
    state = "ASK_NAME"
    attempt = 0

    random.seed()
    iterator = 0 # iterates through different parts of a conversation
    
    # While we have user's attention
    while (ros.checkStatus() != "STOP"):
    
        if state == "ASK_NAME":

            speak("Hello, what is your name?")
            state = "RECOG_NAME"

        elif state == "RECOG_NAME":
           
            name = Utterance(ps.listen()).getName()
            
            if name != None :
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
            state = "RECOG_LOCATION"
        
        elif state == "RECOG_LOCATION":
        
            location = Utterance(ps.listen()).getLocation()
            state = "ASK_INTERESTED"

            if location != None:
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
            else:
                speak("That sounds so very very exciting.  However, I can not travel up stairs.")

        elif state == "ASK_CAKE":
            speak("Do you like cake?")
            
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
                state = retry(attempt, "ASK_INTERESTED", "RECOG_CAKE", "How unfortunate.  Perhaps you are wiser than you first seemed.")
            
        elif state == "ASK_MEETING":
            
            speak("Have you ever met a robot before?")
            
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
                state = retry(attempt, "ASK_INTERESTED", "RECOG_MEETING", "Your words confuse me");
                        
        elif state == "ASK_INTERESTED":

            speak("Would you be interested in finding out more about this experiment?")
            state = "RECOG_INTERESTED"

        elif state == "RECOG_INTERESTED":

            response = Utterance(ps.listen())

            if response.containsYes()  == True:
                print "SUCCESS: Ticket printed"
                Printer().requestPrint()
            elif response.containsNo() == True:
                print "UNLUCKY: Ticket not printed"
            else:
                print "SUCCESS: Ticket printed"
                Printer().requestPrint()
            
            speak("It has been nice speaking to you.")
            state = "ASK_NAME"
        elif state == "ERROR":
            print "Error state"
            break
        

# Main functional loop
if __name__ == '__main__':

    ps = PocketSphinx()
    ros = ROSControl()

    rospy.init_node('ros_speech_engine', anonymous=True)
    
    while True:

        if (ros.checkStatus() == "STARTSPEAKING"):
            speak("Hello, my name is CHARLES.  Would you be interested in taking part in an experiment?")
            speak("Please take a ticket")
            ros.resetStatus()
        elif (ros.checkStatus() == "STARTCONVERSING"):
            conversationStateMachine(ps, ros)
            ros.resetStatus()
        else:
            time.sleep(1)



