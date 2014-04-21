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
import math
from maestor.srv import *
from cob_people_detection_msgs.msg import *
'''
This is a waist demo class. It basically when created will handle
the demo. Every time it gets a DetectionArray message it goes through
the call back. 
'''

ID_NUM = 9
MAX_NECK = math.pi / 2
FOV_H = 58 * math.pi / 180
FOV_V = 45 * math.pi / 180
RES_X = 640
RES_Y = 480
RAD_PER_PIX_X = FOV_H / RES_X
RAD_PER_PIX_Y = FOV_V / RES_Y
#number of radians per pixel = 58/640 * pi/180

class waist_demo:

    def __init__(self, label="Ryan"):
        self.INCREMENT = .15 
        self.MAX = 1.57
        self.LABEL = label
        rospy.init_node("face_center_controller")
        rospy.Subscriber("cob_people_detection/face_recognizer/face_recognitions", DetectionArray, self.foundFace)
        self.pos = 0
        self.old = True
        self.count = 0
        rospy.spin()

    def setProps(self, names, properties, values):
        try:
            rospy.wait_for_service('setProperties')
            service = rospy.ServiceProxy('setProperties', setProperties)
            service(names, properties, values)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def getProps(self, names, properties):
        try:
            rospy.wait_for_service('getProperties')
            service = rospy.ServiceProxy('getProperties', getProperties)
            res = service(names, properties)
            self.pos = int(res.properties)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e  
    
    '''
    This is the call back for when a face is found. 
    It is called everytime that the node gets a message on the cob_people_detection/face_recognitizer/face_recognitions
    topic. It goes through the message and looks for the face that matches the name LABEL 
    and then gets the x position of that head. Depending on the head's location it either moves 
    the head to the left, the right, or keeps it centered. 
    '''
    def foundFace(self, detection_array):
        detections = detection_array.detections
        xpos = None
        #Look for the correct name in all of the detected faces
        for detect in detections:
            name = detect.label
            if name == self.LABEL:
                xpos = math.floor(detect.mask.roi.x + detect.mask.roi.width/2)
                break
        print "xpos: " + str(xpos)
        #get current position 
        self.getProps("NKY","position") 
        #This is so that the increments happens and the maestro topic is not overrun 
        self.count += 1                         
        if self.count == 5: 
            self.count = 0
            if xpos == None:
                return
            self.delta = (xpos - (RES_X/2)) * RAD_PER_PIX_X
            if abs(self.delta) > .025:
                position = self.pos + self.delta
            else:
                position = self.pos
            print str(position) 
            if abs(position) > MAX_NECK:
                return
            self.setProps("NKY", "position", str(position))
            
if __name__ == '__main__':
    print "Starting the waist demo"
    if len(sys.argv) == 4:
        print str(sys.argv[1])
        demo = waist_demo(sys.argv[1])
    else:
        demo = waist_demo()
    
