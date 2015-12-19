#!/usr/bin/env python
import rospy
import math
import st
import numpy as np
from std_msgs.msg import String
import time


class Drawer:
    def __init__(self):
        rospy.init_node('drawing_stuff', anonymous = True)
        self.behaviorpub = rospy.Publisher('behaviors_cmd', String, queue_size=10)
        self.armpub = rospy.Publisher('arm_cmd', String, queue_size=10)
        self.armpub.publish("data: set_speed:: 3000")



    def Draw(self, command, x, y, z):
        time.sleep(3)
        #getting into position

        msg = "data: move_to:: " + str(x) + "," + str(y) + "," + str(z+250) + ", " + str(0)
        print "sending: ", msg
        self.armpub.publish(msg)
        time.sleep(2)

        msg = "data: rotate_hand:: " + str(200)
        print "sending: ", msg
        self.armpub.publish(msg)
        time.sleep(1)

        msg = "data: rotate_wrist:: " + str(1000)
        print "sending: ", msg
        self.armpub.publish(msg)
        time.sleep(1)


        if command == "Square":

            #making the square
            i = 0;
            j = 0;

            while i<3:
                if i%2 == 0 and j%2 == 0:
                    msg = "data: move_to:: " + str(x+250) + ", " + str(y+250) + ", " + str(z) + ", " +str(0)
                    i+=1
                elif i%2 == 1 and j%2 == 0:
                    msg = "data: move_to:: " + str(x-250) + ", " + str(y+250) + ", " + str(z) + ", " +str(0) 
                    j+=1
                elif i%2 == 1 and j%2 == 1:
                    msg = "data: move_to:: " + str(x-250) + ", " + str(y-250) + ", " + str(z) + ", " +str(0)  
                    i+=1   
                else:
                    msg = "data: move_to:: " + str(x+250) + ", " + str(y-250) + ", " + str(z) + ", " +str(0)     
                    j+=1
                
                print "sending: ", msg
                self.armpub.publish(msg)
                time.sleep(1)    


        elif command == "Circle":

            #drawing the circle
            msg = "data: move_to:: " + str(x-200) + ", " + str(y) + ", " + str(z+250)+ ", " + str(0)
            print "sending: ", msg
            self.armpub.publish(msg)
            time.sleep(1)

            for i in range(-200, 210, 10):
                msg = "data: move_to:: " + str(x+i) + ", " + str(int(math.sqrt(40000-((x+i)**2)))+3500) + ", " + str(z)+ ", " + str(0)
                print "sending: ", msg
                self.armpub.publish(msg)
                time.sleep(.5)

            for j in range(-200, 210, 10):
                msg = "data: move_to:: " + str(x-j) + ", " + str(int(-1*math.sqrt(40000-((x-j)**2)))+3500) + ", " + str(z)+ ", " + str(0)
                print "sending: ", msg
                self.armpub.publish(msg)
                time.sleep(.5)

            #coloring the circle
            msg = "data: move_to:: " + str(x-200) + ", " + str(y) + ", " + str(z+250)+ ", " + str(0)
            print "sending: ", msg
            self.armpub.publish(msg)
            time.sleep(1)

            for i in range(-200, 200, 10):
                msg = "data: move_to:: " + str(x+i) + ", " + str(int(math.sqrt(40000-((x+i)**2)))+3500) + ", " + str(z)+ ", " + str(0)
                print "sending: ", msg
                self.armpub.publish(msg)
                time.sleep(.5)
                msg = "data: move_to:: " + str(x+i) + ", " + str(int(-1*math.sqrt(40000-((x+i)**2)))+3500) + ", " + str(z)+ ", " + str(0)
                print "sending: ", msg
                self.armpub.publish(msg)
                time.sleep(.5)


        else:
            print "Not a valid command. Try Again."    



        #picking up off the paper and finishing
        msg = "data: move_to:: " + str(x) + ", " + str(y) + ", " + str(z+250) + ", " + str(0)
        print "sending: ", msg
        self.armpub.publish(msg)
        time.sleep(1)


    def run(self):
         r = rospy.Rate(10)
         while not rospy.is_shutdown():
             r.sleep()



if __name__ == "__main__":
    draw = Drawer()
    #draw.run()
    draw.Draw("Square", 0, 3500, -670)
    draw.Draw("Circle", 0, 3500, -670)
    rospy.spin()
