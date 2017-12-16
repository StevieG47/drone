'''
outline of the code.
move()
main loop{
do move according to status
}

callback loop{

if y > 3.0m
    status 1:
            move drone till y=3

if y < 3.5 and y > 2.0m
    status 2:
            fix yaw
            make x = 0
            make z = 0
            make y = 2

if y < 2.5m:
    status 3:
            fix yaw
            make x = 0
            make z = 0
            just hope it flies through.
}
'''
#create it
#set center
#

import roslib
import rospy
import tf
import math
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Char
from std_msgs.msg import Empty
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from ardrone_autonomy.msg import Navdata
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker

class Drone:


    def __init__(self):
        self.vel_y = 0
        self.vel_x = 0
        self.vel_z = 0
        self.vel_yaw = 0
        self.g_y = 0
        self.g_x = 0
        self.g_z = 0
        self.y = 0
        self.x = 0
        self.z = 0
        self.yaw = 0
        self.final_stage = False

            ############################################
            #           TUNE THESE PARAMS
            ############################################

        self.x_dist_acc_stage_1 = 3.0
        self.x_dist_acc_stage_2 = 1.0
        self.stage_x_buffer = .5
        self.stage = 0

        self.x_goal_stage_1 = 3 #4
        self.x_goal_stage_2 = 2.5 #3
        self.x_goal_stage_3 = 2.0 #2.5

        self.y_goal = 0
        self.z_goal = 0
        self.yaw_goal = 0

        self.y_pos_acc_stage_1 = .25
        self.x_pos_acc_stage_1 = 0

        self.y_pos_acc_stage_2 = .5
        self.x_pos_acc_stage_2 = 0
        self.z_pos_acc_stage_2 = .5
        self.yaw_pos_acc_stage_2 = 10
        self.yaw_acc_stage_2 = 10


        self.y_pos_acc_stage_3 = .05
        self.x_pos_acc_stage_3 = 0
        self.z_pos_acc_stage_3 = .25
        self.yaw_pos_acc_stage_3 = 10
        self.yaw_acc_stage_3 = 10

        self.speed_stage_1 = .1
        self.speed_stage_2 = .03
        self.speed_stage_3 = .03

    def set_vel(self,y,x,z, yaw):
        self.vel_y = y
        self.vel_x = x
        self.vel_z = z
        self.vel_yaw = yaw


    def set_goal(self,y,x,z):
        self.g_y = y
        self.g_x = x
        self.g_z = z
        self.update_stage()


    def set_center(self,y,x,z,yaw):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw


    def update_goal(self):
        if self.stage == 1:
            self.g_x = self.x_goal_stage_1

        elif self.stage == 2:
            self.g_x = self.x_goal_stage_2
            self.g_y = self.y_goal
            self.g_z = self.z_goal
            self.g_yaw = self.yaw_goal

        elif self.stage == 3:
            self.g_x = self.x_goal_stage_3
            self.g_y = self.y_goal
            self.g_z = self.z_goal
            self.g_yaw = self.yaw_goal


    def update_stage(self):
        prev_stage = self.stage

        if self.x > self.x_goal_stage_1 and (self.stage == 1 or self.stage == 0):
            self.stage = 1
        elif self.x > self.x_goal_stage_2 and (self.stage == 1 or self.stage == 2 or self.stage == 0):
            self.stage = 2
        elif self.x_goal_stage_2 > self.x and (self.stage == 2 or self.stage == 3 or self.stage == 0):
            self.stage = 3

        if prev_stage - self.stage != 0:
            self.update_goal()


    def update(self,x,y,z,yaw):
        self.set_center(y,x,z,yaw)
        self.update_stage()

        if not self.final_stage:

            if self.stage == 1:
                self.vel_x = 0
                self.vel_y = 0
                self.vel_z = 0
                self.vel_yaw = 0
                # adjust x velocity
                if abs(self.x - self.g_x )> self.x_pos_acc_stage_1:
                    #rospy.loginfo('in stage 1: fixing x ' + str(self.g_x))
                    if self.x > self.g_x:
                        self.vel_x =  self.speed_stage_1# may need to change this -------------
                    else:
                        self.vel_x = 0
                else:
                    self.vel_x = 0

                    #rospy.loginfo('in stage 1: fixed x ' + str(self.g_x))


                # if abs(self.x - self.g_x > self.x_pos_acc_stage_1):
                #     if self.x > self.g_x:
                #         self.vel_x = -1 * self.speed_stage_1 # may need to change this -------------
                #     else:
                #         self.vel_x = self.speed_stage_1
                # else:
                #     self.vel_x = 0

            elif self.stage == 2:
                self.vel_x = 0
                self.vel_y = 0
                self.vel_z = 0
                self.vel_yaw = 0
                # adjust y velocity
                if abs(self.y - self.g_y )> self.y_pos_acc_stage_2:
                    #rospy.loginfo('in stage 2: fixing y ' + str(self.g_y))
                    if self.y > self.g_y:
                        self.vel_y = 1 * self.speed_stage_2 # may need to change this -------------
                    else:
                        self.vel_y = -1 * self.speed_stage_2
                else:
                    self.vel_y = 0
                    #rospy.loginfo('in stage 2: fixed y  ' + str(self.y))

                    #adjust z
                    if abs(self.z - self.g_z )> self.z_pos_acc_stage_2:
                        if self.z > self.g_z:
                    #        rospy.loginfo('in stage 2: fixing z  ' + str(self.g_z))
                            self.vel_z = 1 * self.speed_stage_2 # may need to change this -------------
                        else:
                            self.vel_z = -1*self.speed_stage_2
                    else:
                        self.vel_z = 0
                    #    rospy.loginfo('in stage 2: fixed z  ' + str(self.z))

                    #     #adjust yaw
                    #     if abs(self.yaw - 0 )> self.yaw_acc_stage_2:
                    # #        rospy.loginfo('in stage 2: fixing yaw  ' + str(0))
                    #         if self.yaw > 0:
                    #             self.vel_yaw = 1 * self.speed_stage_2 # may need to change this -------------
                    #         else:
                    #             self.vel_yaw = -1 *self.speed_stage_2
                    #     else:
                    #         self.vel_yaw = 0
                    #        rospy.loginfo('in stage 2: fixed yaw  ' + str(self.yaw))

                        # adjust x
                        if abs(self.x - self.g_x) > self.x_pos_acc_stage_2:
                #            rospy.loginfo('in stage 2: fixing x  ' + str(self.g_x))
                            if self.x > self.g_x:
                                self.vel_x = 1 * (self.speed_stage_2) # may need to change this -------------
                            else:
                                self.vel_x = 0
                        else:
                            self.vel_x = 0
                #            rospy.loginfo('in stage 2: fixed x  ' + str(self.x))

                            #all adjusted.


                #TO DO


            elif self.stage == 3:
                self.vel_x = 0.02
                self.vel_y = 0
                self.vel_z = 0
                self.vel_yaw = 0


                if abs(self.y - self.g_y )> self.y_pos_acc_stage_3:
                    #rospy.loginfo('in stage 2: fixing y ' + str(self.g_y))
                    if self.y > self.g_y:
                        self.vel_y = 1 * self.speed_stage_3 # may need to change this -------------
                    else:
                        self.vel_y = -1 * self.speed_stage_3
                else:
                    self.vel_y = 0
                    #rospy.loginfo('in stage 2: fixed y  ' + str(self.y))

                    #adjust z
                    if abs(self.z - self.g_z )> self.z_pos_acc_stage_3:
                        if self.z > self.g_z:
                    #        rospy.loginfo('in stage 2: fixing z  ' + str(self.g_z))
                            self.vel_z = 1 * self.speed_stage_3 # may need to change this -------------
                        else:
                            self.vel_z = -1*self.speed_stage_3
                    else:
                        self.vel_z = 0
                        rospy.loginfo('in stage 3: fixed z  ' + str(self.z))

                        #adjust yaw
                        if abs(self.yaw - 0 )> self.yaw_acc_stage_3:
                    #        rospy.loginfo('in stage 2: fixing yaw  ' + str(0))
                            if self.yaw > 0:
                                self.vel_yaw = 1 * self.speed_stage_3 # may need to change this -------------
                            else:
                                self.vel_yaw = -1 *self.speed_stage_3
                        else:
                            self.vel_yaw = 0
                            rospy.loginfo('in stage 3: fixed yaw  ' + str(self.yaw))

                            # adjust x
                            if abs(self.x - self.g_x) > self.x_pos_acc_stage_3:
                    #            rospy.loginfo('in stage 2: fixing x  ' + str(self.g_x))
                                if self.x > self.g_x:
                                    self.vel_x = 1 * self.speed_stage_3 # may need to change this -------------
                                else:
                                    self.vel_x = 0
                            else:
                                self.vel_x = 0

                                stage_final = True
                                rospy.loginfo('#######################final stage###########################')



                            #rospy.loginfo('in stage 2: fixed x  ' + str(self.x))

                            #all adjusted.



    def get_center(self):
        return [self.x,self.y,self.z]

    def get_vel(self):
        return [self.vel_x,self.vel_y,self.vel_z, self.vel_yaw]
