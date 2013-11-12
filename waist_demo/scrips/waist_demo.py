#!/usr/bin/env python
'''
This node is run in conjunction with the cob_people_detection package. When run,
it listens to see if a face is tracked and when a face matching the label 
LABEL is found it trys to center hubo's upper body on the face.
'''

import roslib; roslib.load_manifest('waist_demo')
import rospy
from hubomsg import *
from cob_people_detection_msgs import *

class waist_demo:

    def __init__(self, label=Ryan):
    self.LABEL = label
    rospy.init_node("face_center_controller")
    rospy.Subscriber("cob_people_detection/face_recognizer/face_recognitions", DetectionArray, self.foundFace)
    rospy.Subscriber("Maestro/Message", MaestroMessage, self.updatePos)
    self.pub = rospy.Publisher("Maestro/Control", PythonMessage)
    self.pos = 0
    rospy.spin()

    def foundFace(self, detection_array):
    detections = detection_array.detections
    xpos = 0
    self.pub.publish("WST", "Get", "0", "position")
    for detect in detections:
        name = detect.label
        if name == self.LABEL:
            xpos = detect.mask.roi.x
            break
    if x < -.01:
        self.IncrementRight()
    elif x > .01:
        self.IncrementLeft()
    else:
        Stop()
    

    def IncrementRight(self):
    self.pub.publish("WST", "position", str(self.pos + .05), "")
    

    def IncrementLeft(self):
    self.pub.publish("WST", "position", str(self.pos - .05), "")

    def Stop(self):
    if self.old:
        pass
    else:
        self.pub.publish("WST", "position", str(self.pos), "")

    def updatePos(self, message):
    oldPos = self.pos
    self.pos = message.value
    if oldPos == self.pos:
        self.old = True
    else
        self.old = False

if __name__ == '__main__':
    print "Starting the waist demo"
    demo = waist_demo()
    
