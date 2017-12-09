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

class Drone:

    ############################################
    #           TUNE THESE PARAMS
    ############################################
    y_dist_acc_stage_1 = 3.0
    y_dist_acc_stage_2 = 2.0
    stage_y_buffer = .5
    stage = 0

    y_goal_stage_1 = 3
    y_goal_stage_2 = 2
    y_goal_stage_3 = 2

    x_goal = 0
    z_goal = 0
    yaw_goal = 0

    x_pos_acc_stage_1 = .5
    y_pos_acc_stage_1 = .5

    x_pos_acc_stage_2 = .2
    y_pos_acc_stage_2 = .2
    z_pos_acc_stage_2 = .2
    yaw_pos_acc_stage_2 = 10


    x_pos_acc_stage_3 = .15
    y_pos_acc_stage_3 = .15
    z_pos_acc_stage_3 = .15

    speed_stage_1 = .2
    speed_stage_2 = .1
    speed_stage_3 = .1

    def set_vel(self,x,y,z, yaw):
        self.vel_x = x
        self.vel_y = y
        self.vel_z = z
        self.vel_yaw = yaw


    def set_goal(self,x,y,z):
        self.g_x = x
        self.g_y = y
        self.g_z = z
        self.update_stage(self)


    def set_center(self,x,y,z, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw


    def update_goal(self):
        if self.stage == 1:
            self.g_y = y_goal_stage_1

        elif self.stage == 2:
            self.g_y = y_goal_stage_2
            self.g_x = x_goal
            self.g_z = z_goal
            self.g_yaw = yaw_goal

        elif self.stage == 3:
            self.g_y = y_goal_stage_3
            self.g_x = x_goal
            self.g_z = z_goal
            self.g_yaw = yaw_goal


    def update_stage(self):
        prev_stage = self.stage


        if self.y > self.y_goal_stage_1 and self.stage == 1 or self.stage == 0:
            self.stage = 1
        elif self.y > self.y_goal_stage_2 and self.stage == 1 or self.stage == 2:
            self.stage = 2
        elif self.y_goal_stage_2 > self.y and self.stage == 2 or self.stage == 3:
            self.stage = 3

        if prev_stage - self.stage != 0:
            self.update_goal(self)


    def update(self,x,y,z,yaw):
        self.set_center(self,x,y,z,yaw)
        self.update_stage(self)

        if self.stage == 1:
            if abs(self.y - self.g_y )> self.y_pos_acc_stage_1:
                rospy.loginfo('in stage 1: fixing y ' + str(self.g_y))
                if self.y > self.g_y:
                    self.vel_y = -1 * self.speed_stage_1# may need to change this -------------
                else:
                    self.vel_y = self.speed_stage_1
            else:
                self.vel_y = 0

                rospy.loginfo('in stage 1: fixed y ' + str(self.g_y))


            # if abs(self.x - self.g_x > self.x_pos_acc_stage_1):
            #     if self.x > self.g_x:
            #         self.vel_x = -1 * self.speed_stage_1 # may need to change this -------------
            #     else:
            #         self.vel_x = self.speed_stage_1
            # else:
            #     self.vel_x = 0

        elif self.stage == 2:
            if abs(self.x - self.g_x )> self.x_pos_acc_stage_2:
                rospy.loginfo('in stage 2: fixing x ' + str(self.g_x))
                if self.x > self.g_x:
                    self.vel_x = -1 * self.speed_stage_2 # may need to change this -------------
                else:
                    self.vel_x = self.speed_stage_2
            else:
                self.vel_x = 0
                rospy.loginfo('in stage 2: fixed x  ' + str(self.x))

                #adjust z
                if abs(self.z - self.g_z )> self.z_pos_acc_stage_2:
                    if self.z > self.g_z:
                        rospy.loginfo('in stage 2: fixing z  ' + str(self.g_z))
                        self.vel_z = -1 * self.speed_stage_2 # may need to change this -------------
                    else:
                        self.vel_z = self.speed_stage_2
                else:
                    self.vel_z = 0
                    rospy.loginfo('in stage 2: fixed z  ' + str(self.z))

                    #adjust yaw
                    if abs(self.yaw - 0 )> self.yaw_acc_stage_2:
                        rospy.loginfo('in stage 2: fixing yaw  ' + str(0))
                        if self.yaw > 0:
                            self.vel_yaw = -1 * self.speed_stage_2 # may need to change this -------------
                        else:
                            self.vel_yaw = self.speed_stage_2
                    else:
                        self.vel_yaw = 0
                        rospy.loginfo('in stage 2: fixed yaw  ' + str(self.yaw))

                        #adjust Y
                        if abs(self.y - self.g_y) > self.y_pos_acc_stage_2:
                            rospy.loginfo('in stage 2: fixing y  ' + str(self.g_y))
                            if self.y > self.g_y:
                                self.vel_y = -1 * self.speed_stage_2 # may need to change this -------------
                            else:
                                self.vel_y = self.speed_stage_2
                        else:
                            self.vel_y = 0
                            rospy.loginfo('in stage 2: fixed y  ' + str(self.y))

                            #all adjusted.


            #TO DO


        elif self.stage == 3:
            pass
            #TO DO


    def get_center(self):
        return [self.x,self.y,self.z]

    def get_vel(self):
        return [self.vel_x,self.vel_y,self.vel_z, self.vel_yaw]
