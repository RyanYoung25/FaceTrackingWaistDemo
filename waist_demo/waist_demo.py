#!/usr/bin/env python
'''
This node is run in conjunction with the cob_people_detection package. When run,
it listens to see if a face is tracked and when a face matching the label 
LABEL is found it trys to center hubo's upper body on the face.

Author: Ryan
'''

import roslib; roslib.load_manifest('waist_demo')
import rospy
import time
import sys
from hubomsg.msg import *
from cob_people_detection_msgs.msg import *
'''
This is a waist demo class. It basically when created will handle
the demo. Every time it gets a DetectionArray message it goes through
the call back. 
'''

ID_NUM = 9
class waist_demo:

    def __init__(self, label="Ryan"):
        self.INCREMENT = .15 
        self.MAX = 1.57
        self.LABEL = label
        rospy.init_node("face_center_controller")
        rospy.Subscriber("cob_people_detection/face_recognizer/face_recognitions", DetectionArray, self.foundFace)
        rospy.Subscriber("Maestro/Message", MaestroMessage, self.updatePos)
        self.pub = rospy.Publisher("Maestro/Control", MaestroCommand)
        self.pos = 0
        self.old = True
        self.count = 0
        rospy.spin()
    
    '''
    This is the call back for when a face is found. 
    It is called everytime that the node gets a message on the cob_people_detection/face_recognitizer/face_recognitions
    topic. It goes through the message and looks for the face that matches the name LABEL 
    and then gets the x position of that head. Depending on the head's location it either moves 
    the head to the left, the right, or keeps it centered. 
    '''
    def foundFace(self, detection_array):
        detections = detection_array.detections
        xpos = 0
        #Look for the correct name in all of the detected faces
        for detect in detections:
            name = detect.label
            if name == self.LABEL:
                xpos = detect.mask.roi.x
                break
        print "xpos: " + str(xpos)
        self.pub.publish("NKY", "Get", "0", "position", ID_NUM)
        #This is so that the increments happens and the maestro topic is not overrun 
        self.count += 1                         
        if self.count == 5: 
            self.count = 0
            #Checks against a threshold
            #TODO: make the 240 and 260 not hard coded. 
            #NOTE: look at the cob people tracking to see if there is a default value set in some xml file somewhere! 
            if xpos < 140 and xpos != 0:
                self.IncrementRight()
            elif xpos > 160:
                self.IncrementLeft()
            else:
                self.Stop(xpos)
    

    '''
    Tell's maestro to move the wst the increment to the right from the current 
    position
    '''
    def IncrementRight(self):
        position = self.pos + self.INCREMENT
        if position > self.MAX:
            position = self.MAX
        print str(position)
        self.pub.publish("NKY", "position", str(position), "", ID_NUM)
    
    '''
    Tell's maestro to move the wst the increment to the left from the current 
    position
    '''
    def IncrementLeft(self):
        position = self.pos - self.INCREMENT
        if position < -self.MAX:
            position = -self.MAX
        print str(position)
        self.pub.publish("NKY", "position", str(position), "", ID_NUM)

    '''
    Does nothing but might have to do something in the future.
    '''
    def Stop(self, position):
        if position == 0:
            print self.LABEL + " is not in the frame!"
        else:
            print self.LABEL + " is centered!"

    '''
    The call back for the maestro message channel. It just updates the position
    that robot is currently at. 
    '''
    def updatePos(self, message):
        if message.id == ID_NUM:
            self.pos = message.value

if __name__ == '__main__':
    print "Starting the waist demo"
    if len(sys.argv) == 4:
        print str(sys.argv[1])
        demo = waist_demo(sys.argv[1])
    else:
        demo = waist_demo()
    
