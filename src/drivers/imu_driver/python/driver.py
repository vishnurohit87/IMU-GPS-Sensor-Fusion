#!/usr/bin/env python3

import rospy
import sys
import utm
from pyproj import Proj
import serial
from math import sin, pi, cos, radians, degrees
from std_msgs.msg import Float64
import geometry_msgs.msg
from imu_driver.msg import imu_msg

def eu2qtn(r,p,y): #ZYX
    cr=cos(r/2)
    sr=sin(r/2)
    cp=cos(p/2)
    sp=sin(p/2)
    cy=cos(y/2)
    sy=sin(y/2)
    q=[sr*cp*cy-cr*sp*sy , cr*sp*cy+sr*cp*sy , cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy] #x,y,z,w
    return q
#Qua=eu2qtn(1,2,3)
#print(Qua)


def imuNode():
    pub = rospy.Publisher('imu', imu_msg, queue_size=10)
    rospy.init_node('imuNode', anonymous=True)           #node inititalization
    serial_port=sys.argv[1]
    print(serial_port)
    serial_baud=115200
    imu=imu_msg()                                                   #inititalize gps_msg classs as an ovject to call it
    port = serial.Serial(serial_port, serial_baud, timeout=1)
    port.write(b'$VNWRG,07,10*XX')                              #Set frequency to 40Hz

    while not rospy.is_shutdown():
        line=port.readline().strip().decode("utf-8")
        print(line)
        if "VNYMR" in line:
            imuls=line.split(',')
            print(imuls)
            [r,p,y]=[radians(float(imuls[3])),radians(float(imuls[2])),radians(float(imuls[1]))]                    #Extract Roll Pitch Yaw
            [Mx,My,Mz]=[float(imuls[4]),float(imuls[5]),float(imuls[6])]
            [Ax,Ay,Az]=[float(imuls[7]),float(imuls[8]),float(imuls[9])]
            [Gx,Gy,Gz]=[imuls[10],imuls[11],imuls[12][0:imuls[12].find('*')]]      #Remove checksum at the end
            #print("RPY",r,p,y,"\nMxyz",Mx,My,Mz,"\nAxyz",Ax,Ay,Az,"\nGxyz",Gx,Gy,Gz)
            Quat=eu2qtn(r,p,y)
            #print("Quaternions: ",Quat)
            time = ltime=rospy.Time.now()
            print('\n')

            imu.Header.stamp.secs=time.secs
            imu.Header.stamp.nsecs=time.nsecs
            imu.Header.frame_id='IMU1_Frame'
            imu.IMU.orientation.x=float(Quat[0])
            imu.IMU.orientation.y=float(Quat[1])
            imu.IMU.orientation.z=float(Quat[2])
            imu.IMU.orientation.w=float(Quat[3])
            imu.IMU.angular_velocity.x=float(Gx)
            imu.IMU.angular_velocity.y=float(Gy)
            imu.IMU.angular_velocity.z=float(Gz)
            imu.IMU.linear_acceleration.x=float(Ax)
            imu.IMU.linear_acceleration.y=float(Ay)
            imu.IMU.linear_acceleration.z=float(Az)
            imu.MagField.magnetic_field.x=float(Mx)
            imu.MagField.magnetic_field.y=float(My)
            imu.MagField.magnetic_field.z=float(Mz)
            imu.IMUstr=str(line)


            print('===================== IMU Data Sent ====================\n',imu)
            print("\n\n\n")

            pub.publish(imu)


imuNode()
