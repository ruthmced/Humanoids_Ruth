#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math
import numpy as np


class MadgwickFilterROS:
    def __init__(self):
        self.orientation_publisher = rospy.Publisher('/imu/orientation', Quaternion, queue_size=10)
        self.imu_subscriber = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # Inicialización de la orientación cuaternión
        self.beta = 0.1  # Parámetro beta del filtro de Madgwick
        #self.dt = 0.01
        self.previous_time = rospy.Time.now()

    def imu_callback(self, msg):
        # Obtener las mediciones del IMU
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        # Convertir las mediciones a radianes por segundo
        gx *= math.pi / 180.0
        gy *= math.pi / 180.0
        gz *= math.pi / 180.0

        # Calculate time interval (dt) since the previous IMU message
        current_time = msg.header.stamp
        dt = (current_time - self.previous_time).to_sec()

        # Calcular la orientación utilizando el filtro de Madgwick
        self.update_filter(ax, ay, az, gx, gy, gz, dt)

        # Publicar la orientación en formato Quaternion
        quaternion_msg = Quaternion()
        quaternion_msg.x = self.q[1]
        quaternion_msg.y = self.q[2]
        quaternion_msg.z = self.q[3]
        quaternion_msg.w = self.q[0]
        self.orientation_publisher.publish(quaternion_msg)

    def update_filter(self, ax, ay, az, gx, gy, gz, dt):
        # Normalizar las mediciones de aceleración
        norm_acc = math.sqrt(ax * ax + ay * ay + az * az)
        if norm_acc == 0.0:
            return
        ax /= norm_acc
        ay /= norm_acc
        az /= norm_acc

        # Calcular los cuaterniones auxiliares
        q1 = self.q[0]
        q2 = self.q[1]
        q3 = self.q[2]
        q4 = self.q[3]

        # Gradient descent algorithm
        f1 = 2.0 * (q2 * q4 - q1 * q3) - ax
        f2 = 2.0 * (q1 * q2 + q3 * q4) - ay
        f3 = 2.0 * (0.5 - q2 * q2 - q3 * q3) - az
        f4 = 2.0 * self.beta * (q2 * f3 - q3 * f2) - gx
        f5 = 2.0 * self.beta * (q3 * f1 - q1 * f3) - gy
        f6 = 2.0 * self.beta * (q1 * f2 - q2 * f1) - gz

        # Actualizar los cuaterniones
        self.q[0] += (-q2 * f4 - q3 * f5 - q4 * f6) * dt
        self.q[1] += (q1 * f4 + q3 * f6 - q4 * f5) * dt
        self.q[2] += (q1 * f5 - q2 * f6 + q4 * f4) * dt
        self.q[3] += (q1 * f6 + q2 * f5 - q3 * f4) * dt
        
        # Normalizar los cuaterniones
        norm_quat = math.sqrt(self.q[0] * self.q[0] + self.q[1] * self.q[1] + self.q[2] * self.q[2] + self.q[3] * self.q[3])
        self.q /= norm_quat

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('madgwick_filter')
    madgwick_filter = MadgwickFilterROS()
    madgwick_filter.run()


#En este ejemplo, se crea una clase MadgwickFilterROS que implementa el filtro de Madgwick en Python y se integra con ROS.

#El filtro de Madgwick se aplica a las mediciones de aceleración y velocidad angular provenientes del IMU (Unidad de Medición Inercial). Estas mediciones se obtienen de un topic llamado /imu/data utilizando el objeto Imu de sensor_msgs.msg. La orientación filtrada se publica en formato Quaternion en el topic /imu/orientation utilizando el objeto Quaternion de geometry_msgs.msg.

#El filtro de Madgwick se actualiza en el método update_filter() utilizando el algoritmo de descenso de gradiente. Se utilizan los parámetros beta y frequency que se obtienen de los parámetros del nodo de ROS.

#El método run() inicia el bucle principal de ROS para recibir y procesar los mensajes de IMU.

#Recuerda ajustar los nombres de los topics y los parámetros según la configuración específica de tu sistema ROS.

#Este código proporciona una base para implementar el filtro de Madgwick en Python dentro del entorno de ROS y puede ser adaptado según tus necesidades y requisitos específicos.