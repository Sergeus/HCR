from std_msgs.msg import String
import os

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