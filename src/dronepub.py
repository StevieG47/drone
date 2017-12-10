#!/usr/bin/env python
# license removed for brevity
import roslib
import rospy
import math
import tf
import collections
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

from drone import Drone


#Variables initialized
cmd_vel = Twist()
nav_data = Navdata()
imu_data = Imu()
odom = Odometry()
empty=Empty()
ar_data = AlvarMarkers()
# queues for keeping center positions to take average
mean_center_x = collections.deque(maxlen=20)
mean_center_y = collections.deque(maxlen=20)
mean_center_z = collections.deque(maxlen=20)
mean_center_yaw = collections.deque(maxlen=20)

# TODO set it to false if you want to fly the drone
testing = True

cx = 0.0
cy = 0.0
cz = 0.0

markerFlag = 0 # if we see the marker
statusFlag = 1
adjHeight = 0
adjX = 0
adjY = 0
init_goalx = 0
init_goaly = 0
init_goalz = 0
fist_sighting = True
drone = Drone()

mean_vel_x = 0
mean_vel_y = 0
mean_vel_z = 0
mean_vel_yaw = 0

# variables which keep the number of lost tags
fuck = 0
reset_fuck = 0

center_pub = rospy.Publisher('center_co', Vector3, queue_size=10, latch=True)

#pseudo publisher for velocity
vel_pub_test = rospy.Publisher('cmd_vel_test', Twist, queue_size=10, latch=True)

vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10, latch=True)

# quoternion to rotation matrix
def quatToRot(x, y, z, w):
    R = np.array([[1-2*y**2-2*z**2, 2*x*y-2*z*w, 2*x*z+2*y*w],
                  [2*x*y+2*z*w, 1-2*x**2-2*z**2, 2*y*z-2*x*w],
                  [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x**2-2*y**2]])
    return R

def quaternion_to_euler_angle(w, x, y, z):
	ysqr = y * y

	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + ysqr)
	X = math.degrees(math.atan2(t0, t1))

	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	Y = math.degrees(math.asin(t2))

	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (ysqr + z * z)
	Z = math.degrees(math.atan2(t3, t4))

	return X, Y, Z


def main():
    #Intialize the ROS Node
    rospy.init_node('drone_control', anonymous=True)

    #pseudo publisher for velocity
    vel_pub_test = rospy.Publisher('cmd_vel_test', Twist, queue_size=10, latch=True)

    takeoff_pub = rospy.Publisher('ardrone/takeoff', Empty, queue_size=1, latch=True)
    reset_pub = rospy.Publisher('ardrone/reset', Empty, queue_size=1, latch=True)

    #TF listener initialized
    listener = tf.TransformListener()

    #Reset the drone
    land_pub = rospy.Publisher('ardrone/land', Empty, queue_size=1, latch=True)
    rospy.loginfo("Drone Resetting: Please step away")
    reset_pub.publish(empty)
    rospy.sleep(1.0)
    reset_pub.publish(empty)
    rospy.sleep(1.0)

    if not testing:
        reset_pub.publish(empty)
        rospy.sleep(1.0)

    #Takeoff the drone first
    rospy.loginfo("Drone taking off")
    takeoff_pub.publish(empty)
    rospy.sleep(3.0)

    if not testing:
        takeoff_pub.publish(empty)
        rospy.sleep(5.0)


    #The Control loop for navigation
    while not rospy.is_shutdown():
        rospy.sleep(1.0)
        rospy.Subscriber("ardrone/navdata", Navdata, nav_callback)
        rospy.Subscriber("ardrone/odometry", Odometry, odom_callback)
        rospy.Subscriber("ardrone/imu", Imu, imu_callback)
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, ar_callback )

        #rospy.loginfo(['in main X', adjX, ' Y', adjY, ' Z', adjHeight, ' Marker', markerFlag])

        if markerFlag == 0:
            rospy.loginfo("Searching")
            # TODO change the = to fix initial angle
            if np.mean(mean_center_yaw) >= 0:
                cmd_vel.angular.z  = -0.2
            elif np.mean(mean_center_yaw) < 0:
                cmd_vel.angular.z  = 0.2
            else:
                cmd_vel.angular.z  = 0.1

            rospy.loginfo("Rotating")
            vel_pub.publish(cmd_vel)

            if not testing:
                vel_pub.publish(cmd_vel)
                rospy.sleep(0.1) #need to adjust %%%%%%%%%%%%%%%%

def ar_callback(data):
    global markerFlag
    global statusFlag
    global adjHeight
    global adjX
    global adjY
    global centerx
    global centery
    global centerz
    global init_goalx
    global init_goaly
    global init_goalz
    global first_sighting
    global drone
    global yaw
    global fuck
    global reset_fuck
    global center_pub
    global vel_pub
    global mean_vel_x
    global mean_vel_y
    global mean_vel_z
    global mean_vel_yaw
    global mean_center_x
    global mean_center_y
    global mean_center_z
    global mean_center_yaw

    ar_data = data
    rot = np.array([0.0150814330344, -0.020624200866, -0.544323169626, 0.838486421908])
    trans = np.array([-0.0841731084939, 0.133747005225, 0.0])
    #rospy.sleep(.1)
    #rospy.loginfo(ar_data.markers)
    #rospy.loginfo(ar_data)
    #rospy.loginfo(len(ar_data.markers))

    #rospy.loginfo(['in callback X', adjX, ' Y', adjY, ' Z', adjHeight, ' Marker', markerFlag])


    if len(ar_data.markers) >= 4:
        fuck = fuck + 1
        t0 = ar_data.markers[0].pose.pose.position
        t1 = ar_data.markers[1].pose.pose.position
        t2 = ar_data.markers[2].pose.pose.position
        t3 = ar_data.markers[3].pose.pose.position

        quat_0 = ar_data.markers[0].pose.pose.orientation
        quat_1 = ar_data.markers[1].pose.pose.orientation
        quat_2 = ar_data.markers[2].pose.pose.orientation
        quat_3 = ar_data.markers[3].pose.pose.orientation

        a, yaw_0, b = quaternion_to_euler_angle(quat_0.w, quat_0.x, quat_0.y, quat_0.z)
        a, yaw_1, b = quaternion_to_euler_angle(quat_1.w, quat_1.x, quat_1.y, quat_1.z)
        a, yaw_2, b = quaternion_to_euler_angle(quat_2.w, quat_2.x, quat_2.y, quat_2.z)
        a, yaw_3, b = quaternion_to_euler_angle(quat_3.w, quat_3.x, quat_3.y, quat_3.z)

        #rospy.loginfo(["yaw avg: ", np.mean([yaw_0, yaw_1, yaw_2, yaw_3])])
        yaw = np.mean([yaw_0, yaw_1, yaw_2, yaw_3])

        cy = -1*(t0.x + t1.x + t2.x + t3.x)/4
        cz = (t0.y + t1.y + t2.y + t3.y)/4
        cx = (t0.z + t1.z + t2.z + t3.z)/4

        c = np.array([cx, cy, cz])

    # if rotation is quoternion, convert to matrix
        if len(rot) == 4:
            rot = quatToRot(rot[0], rot[1], rot[2], rot[3])
            center = np.dot(rot, c) + trans #might need to adjust


        centerx = c[0]
        centery = c[1]
        centerz = c[2]
        #rospy.loginfo(cz)
        center_pub.publish(Vector3(centerx,centery,centerz))
        mean_center_x.append(centerx)
        mean_center_y.append(centery)
        mean_center_z.append(centerz)
        mean_center_yaw.append(yaw)
        drone.update(np.mean(mean_center_x),np.mean(mean_center_y) , np.mean(mean_center_z), np.mean(mean_center_yaw))

#        rospy.loginfo('current stage: ' + str(drone.stage) + ' current vel: ' + str(drone.get_vel()))

        reset_fuck = 0
        markerFlag = 1
        cmd_vel.angular.z  = 0.0
        mean_vel_x = (mean_vel_x + drone.get_vel()[0])
        mean_vel_y = (mean_vel_y + drone.get_vel()[1])
        mean_vel_z = (mean_vel_z + drone.get_vel()[2])
        mean_vel_yaw = (mean_vel_yaw + drone.get_vel()[3])

        if fuck == 20:
            rospy.loginfo('current stage: ' + str(drone.stage) + ' current vel: ' + str(drone.get_vel()) + 'current yaw' + str(yaw) )
            fuck = 0
            cmd_vel.linear.x = (mean_vel_x/20)
            cmd_vel.linear.y = (mean_vel_y/20)
            cmd_vel.linear.z = (mean_vel_z/20)
            cmd_vel.angular.z = (mean_vel_yaw/20)
            vel_pub_test.publish(cmd_vel)
            mean_vel_x = 0
            mean_vel_y = 0
            mean_vel_z = 0
            mean_vel_yaw = 0

            #rospy.sleep(0.1)


        if drone.stage == 1:
            vel_pub.publish(cmd_vel)

    elif len(ar_data.markers) >= 3:
        fuck = fuck + 1
        t0 = ar_data.markers[0].pose.pose.position
        t1 = ar_data.markers[1].pose.pose.position
        t2 = ar_data.markers[2].pose.pose.position
        #t3 = ar_data.markers[3].pose.pose.position

        quat_0 = ar_data.markers[0].pose.pose.orientation
        quat_1 = ar_data.markers[1].pose.pose.orientation
        quat_2 = ar_data.markers[2].pose.pose.orientation
        #quat_3 = ar_data.markers[3].pose.pose.orientation

        a, yaw_0, b = quaternion_to_euler_angle(quat_0.w, quat_0.x, quat_0.y, quat_0.z)
        a, yaw_1, b = quaternion_to_euler_angle(quat_1.w, quat_1.x, quat_1.y, quat_1.z)
        a, yaw_2, b = quaternion_to_euler_angle(quat_2.w, quat_2.x, quat_2.y, quat_2.z)
        #a, yaw_3, b = quaternion_to_euler_angle(quat_3.w, quat_3.x, quat_3.y, quat_3.z)

        #rospy.loginfo(["yaw avg: ", np.mean([yaw_0, yaw_1, yaw_2, yaw_3])])

        # TODO lets make center great again %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        yaw = np.mean([yaw_0, yaw_1, yaw_2])

        # find the ones which are in the same x position
        difference = np.inf
        if abs(t0.x - t1.x) < difference:
            difference = abs(t0.x - t1.x)
            pair = (t0, t1)
            other = t2
        if abs(t0.x - t2.x) < difference:
            difference = abs(t0.x - t2.x)
            pair = (t0, t2)
            other = t1
        if abs(t1.x - t2.x) < difference:
            difference = abs(t1.x - t2.x)
            pair = (t1, t2)
            other = t0
        # compute center position
        cy = -1*(pair[0].x + other.x)/2
        cz = (pair[0].y + pair[1].y)/2
        cx = (pair[0].z + pair[1].z)/2
        c = np.array([cx, cy, cz])

    # if rotation is quoternion, convert to matrix
        if len(rot) == 4:
            rot = quatToRot(rot[0], rot[1], rot[2], rot[3])
            center = np.dot(rot, c) + trans #might need to adjust
        centerx = c[0]
        centery = c[1]
        centerz = c[2]
        #rospy.loginfo(cz)
        center_pub.publish(Vector3(centerx,centery,centerz))
        mean_center_x.append(centerx)
        mean_center_y.append(centery)
        mean_center_z.append(centerz)
        mean_center_yaw.append(yaw)
        drone.update(np.mean(mean_center_x),np.mean(mean_center_y) , np.mean(mean_center_z), np.mean(mean_center_yaw))

#        rospy.loginfo('current stage: ' + str(drone.stage) + ' current vel: ' + str(drone.get_vel()))

        reset_fuck = 0
        markerFlag = 1
        cmd_vel.angular.z  = 0.0
        mean_vel_x = (mean_vel_x + drone.get_vel()[0])
        mean_vel_y = (mean_vel_y + drone.get_vel()[1])
        mean_vel_z = (mean_vel_z + drone.get_vel()[2])
        mean_vel_yaw = (mean_vel_yaw + drone.get_vel()[3])

        if fuck == 20:
            rospy.loginfo('current stage: ' + str(drone.stage) + ' current vel: ' + str(drone.get_vel()))
            fuck = 0
            cmd_vel.linear.x = (mean_vel_x/20)
            cmd_vel.linear.y = (mean_vel_y/20)
            cmd_vel.linear.z = (mean_vel_z/20)
            cmd_vel.angular.z = (mean_vel_yaw/20)
            vel_pub_test.publish(cmd_vel)
            mean_vel_x = 0
            mean_vel_y = 0
            mean_vel_z = 0
            mean_vel_yaw = 0



        if drone.stage == 1:
            vel_pub.publish(cmd_vel)

    else:
        reset_fuck = reset_fuck + 1
        if reset_fuck == 10:
            markerFlag = 0
            reset_fuck = 0
            drone.__init__()
            cmd_vel.linear.x = drone.get_vel()[0]
            cmd_vel.linear.y = drone.get_vel()[1]
            cmd_vel.linear.z = drone.get_vel()[2]
            vel_pub_test.publish(cmd_vel)
            #rospy.sleep(0.1) #need to adjust %%%%%%%%%%%%%%%%

def nav_callback(data):
    nav_data = data

def imu_callback(data):
    imu_data = data

def odom_callback(data):
    odom = data

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
