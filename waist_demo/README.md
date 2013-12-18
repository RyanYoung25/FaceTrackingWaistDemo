FaceTrackingWaistDemo
=====================

This is a ros node that uses the cob_people_detection ros package and the maestro ros package. 
It takes the faces found by the people detector and moves hubo's waist to center him on that person.

To set the Label of the face to center on run the script with the new Label as an argument. 
It only takes one argument so one name at a time. 

Besure to put this package in your rospath if you want to use it on your computer. 

Running rosmake on this package will try to make the packages that it is dependent on. There is no need to build any of the code
in this program because it is just a script. 

You can run this script with the default Person, Ryan, being looked for by running the command:

roslaunch waist_demo demo.launch

To use it with a different person try one of the other launch files or make one of your own by using one 
of the demo ones as a reference. 


