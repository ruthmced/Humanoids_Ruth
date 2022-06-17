#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import Imu

def callbackimu(msg):
    global imu_x
    global imu_y
    global imu_z
    global imu_w

    imu_x = msg.orientation.x
    imu_y = msg.orientation.y
    imu_z = msg.orientation.z
    imu_w = msg.orientation.w

    t0 = +2.0 * (imu_w * imu_x + imu_y * imu_z)
    t1 = +1.0 - 2.0 * (imu_x * imu_x + imu_y * imu_y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (imu_w * imu_y - imu_z * imu_x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (imu_w * imu_z + imu_x * imu_y)
    t4 = +1.0 - 2.0 * (imu_y * imu_y + imu_z * imu_z)
    yaw_z = math.atan2(t3, t4)

    print(roll_x, pitch_y, yaw_z)

    return


def euler_from_quaternion(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z #radianes


#print(euler_from_quaternion(imu_x, imu_y, imu_z, imu_w))


def main():
    rospy.init_node("euler_from_quaternion_test")
    rospy.Subscriber("/imu", Imu, callbackimu)
    loop = rospy.Rate(10)
    rospy.spin()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass