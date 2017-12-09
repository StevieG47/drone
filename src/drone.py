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
    y_dist_acc_stage_1 = 3.0
    y_dist_acc_stage_2 = 2.0
    stage_y_buffer = .5
    stage = 0

    x_pos_acc_stage_1 = 1
    y_pos_acc_stage_1 = 1

    x_pos_acc_stage_2 = .4
    y_pos_acc_stage_2 = .4
    z_pos_acc_stage_2 = .4

    x_pos_acc_stage_2 = .3
    y_pos_acc_stage_2 = .3
    z_pos_acc_stage_2 = .3

    speed_stage_1 = .2
    speed_stage_2 = .1
    speed_stage_3 = .1

    def __init__(self):
        set_vel(self,x,y,z)


    def set_goal(self,x,y,z):
        self.g_x = x
        self.g_y = y
        self.g_z = z
        update_stage(self)


    def set_vel(self,x,y,z):
        self.vel_x = x
        self.vel_y = y
        self.vel_z = z


    def set_center(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z


    def update_goal(self):
        pass # need to implement this.
        #according to the stage call set_goal


    def update_stage(self):
        prev_stage = self.stage
        if self.stage == 0:
            if self.y > self.y_dist_stage_1:
                self.stage = 1
            elif self.y > self.y_dist_stage_2:
                self.stage = 2
            elif self.y < self.y_dist_stage_2:
                self.stage = 3

        if self.y > self.y_dist_stage_1 + stage_y_buffer / 2:
            self.stage = 1
        elif self.y_dist_stage_1 - stage_y_buffer / 2 > self.y and
            self.y > self.y_dist_stage_2 + stage_y_buffer / 2:
            self.stage = 2
        elif self.y_dist_stage_2 - stage_y_buffer / 2 > self.y:
            self.stage = 3

        if prev_stage - self.stage != 0:
            update_goal(self)


    def update(self,x,y,z):
        set_center(self,x,y,z)
        update_stage(self)

        if self.stage == 1:
            if abs(self.y - self.g_y > self.y_pos_acc_stage_1):
                if self.y > self.g_y:
                    self.vel_y = -1 * self.speed_stage_1# may need to change this -------------
                else:
                    self.vel_y = self.speed_stage_1
            else:
                self.vel_y = 0


            if abs(self.x - self.g_x > self.x_pos_acc_stage_1):
                if self.x > self.g_x:
                    self.vel_x = -1 * self.speed_stage_1 # may need to change this -------------
                else:
                    self.vel_x = self.speed_stage_1
            else:
                self.vel_x = 0

        elif self.stage == 2:
            #TO DO


        elif self.stage == 3:
            #TO DO


    def get_center(self):
        return [self.x,self.y,self.z]
