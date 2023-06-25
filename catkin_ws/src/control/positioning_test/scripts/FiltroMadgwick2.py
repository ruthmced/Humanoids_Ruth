#!/usr/bin/env python3
import rospy
from numpy import sqrt 
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Vector3

class MadgwickFilterROS:
    def __init__(self):
        self.orientation_pub = rospy.Publisher('/filtered_orientation', Vector3, queue_size=1)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.quaternion = [1.0, 0.0, 0.0, 0.0]  # Quaternion representing the orientation
        self.beta = 0.1  # Madgwick filter gain
        self.previous_time = rospy.Time.now()

    def imu_callback(self, msg):
        # Extract orientation from IMU message
        orientation = msg.orientation
        imu_quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]

        # Calculate time interval (dt) since the previous IMU message
        current_time = msg.header.stamp
        dt = (current_time - self.previous_time).to_sec()

        # Update the filter
        self.update_filter(imu_quaternion, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, dt)

        # Publish filtered orientation as Euler angles
        euler_angles = euler_from_quaternion(self.quaternion)
        filtered_orientation_msg = Vector3()
        filtered_orientation_msg.x = euler_angles[0]  # Roll
        filtered_orientation_msg.y = euler_angles[1]  # Pitch
        filtered_orientation_msg.z = euler_angles[2]  # Yaw
        self.orientation_pub.publish(filtered_orientation_msg)

        # Update previous time
        self.previous_time = current_time

    def update_filter(self, quaternion, gx, gy, gz, dt):
        q0, q1, q2, q3 = self.quaternion
        q0_imu, q1_imu, q2_imu, q3_imu = quaternion

        # Auxiliary variables to avoid repeated calculations
        q0q0 = q0 * q0
        q0q1 = q0 * q1
        q0q2 = q0 * q2
        q0q3 = q0 * q3
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q3q3 = q3 * q3

        # Gradient descent algorithm
        qDot0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        qDot1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        qDot2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        qDot3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

        # Compute rate of change of quaternion
        qDot0 -= self.beta * q0
        qDot1 -= self.beta * q1
        qDot2 -= self.beta * q2
        qDot3 -= self.beta * q3

        # Integrate to get quaternion
        q0 += qDot0 * dt
        q1 += qDot1 * dt
        q2 += qDot2 * dt
        q3 += qDot3 * dt

        # Normalize quaternion
        norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        self.quaternion = [q0 / norm, q1 / norm, q2 / norm, q3 / norm]

if __name__ == '__main__':
    rospy.init_node('madgwick_filter')
    filter_ros = MadgwickFilterROS()
    rospy.spin()

#Este programa crea una clase MadgwickFilterROS que se suscribe a los mensajes del IMU (Unidad de Medición Inercial) en el tópico /imu/data. Utiliza el filtro de Madgwick para estimar la orientación del IMU y publica los resultados filtrados como ángulos de Euler en el tópico /filtered_orientation. El parámetro beta se puede ajustar para controlar la respuesta del filtro.

#Recuerda que para ejecutar este programa en ROS, debes tener correctamente configurado el entorno de ROS y asegurarte de que los mensajes IMU y los tópicos correspondientes estén configurados adecuadamente.

#Ten en cuenta que este es solo un ejemplo básico y puede requerir ajustes adicionales según tus necesidades y el entorno específico de tu aplicación en ROS.