#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu

def callbackimu(msg):
    global imu_measures_xaxis
    global imu_measures_yaxis
    global imu_measures_zaxis

    imu_measures_zaxis = msg.linear_acceleration.z
    imu_measures_xaxis = msg.linear_acceleration.x
    imu_measures_yaxis = msg.linear_acceleration.y
    #print(imu_measures_xaxis)

    if 9 <= imu_measures_zaxis <= 10:
        print ("Standing")
    elif 9 <= imu_measures_yaxis <= 10:
        print ("On the floor")
    elif 9 <= imu_measures_xaxis <= 10:
        print ("On the floor")
    return

def main():
    rospy.init_node("IMUtest")
    rospy.Subscriber("/imu", Imu, callbackimu)
    loop = rospy.Rate(10)
    rospy.spin()

    global imu_measures_xaxis
    global imu_measures_yaxis
    global imu_measures_zaxis

        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass