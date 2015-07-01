#!/usr/bin/python

import fcntl
import os
import sys
import struct
import socket
from signal import signal, SIGINT, SIG_DFL, SIGTERM
import math as m

import rospy
from robot.msg import *



TCP_IP = '0.0.0.0'
TCP_PORT = 5005
BUFFER_SIZE = 20  # Normally 1024, but we want fast response

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setblocking(0)
s.bind((TCP_IP, TCP_PORT))
# fcntl.fcntl(s, fcntl.F_SETFL, os.O_NONBLOCK)
s.listen(1)

while True:
    try:
        conn, addr = s.accept()
        break
    except Exception as e:
        print e
        continue
print 'Connection address:', addr

fmt = struct.Struct('2B 3h')
fmt2 = struct.Struct('2B3f')
message = [0] * 5

data_dict={}

class DataProcessor:
    def __init__(self):
        self.calibrated=False
        self.data_dict={}
        self.setup_dict()

        self.s_acc=1
        self.s_gyro=4*131*180/m.pi # sensivity+ deg to rad
        self.N_sample=50
        self.alpha=0.98

        self.low_pass=False

    def setup_dict(self):
        self.data_dict["AcX"]=[]
        self.data_dict["AcY"]=[]
        self.data_dict["AcZ"]=[]

        self.data_dict["GyX"]=[]
        self.data_dict["GyY"]=[]
        self.data_dict["GyZ"]=[]


    def on_msgSPI(self, msg):
        self.AcX=msg.imu_acc_x
        self.AcY=msg.imu_acc_y
        self.AcZ=msg.imu_acc_z

        self.GyX=msg.imu_roll
        self.GyY=msg.imu_pitch
        self.GyZ=msg.imu_yaw

        if self.calibrated == False:
            self.data_dict["AcX"].append(self.AcX)
            self.data_dict["AcY"].append(self.AcY)
            self.data_dict["AcZ"].append(self.AcZ)

            self.data_dict["GyX"].append(self.GyX)
            self.data_dict["GyY"].append(self.GyY)
            self.data_dict["GyZ"].append(self.GyZ)

        self.compute()

    def compute(self):
        if self.calibrated==False and len(self.data_dict["AcX"])>self.N_sample+1:
            self.calibrate()

            self.gyro_x=0
            self.gyro_y=0
            self.gyro_z=0

            self.gyro_roll=0
            self.gyro_pitch=0
            self.gyro_yaw=0

            self.roll=0
            self.pitch=0
            self.yaw=0

        if self.calibrated==True:
            dt=0.05

            self.acc_x=(self.AcX-self.acc_x_cal)/self.s_acc
            self.acc_y=(self.AcY-self.acc_y_cal)/self.s_acc
            self.acc_z=(self.AcZ-self.acc_z_cal)/self.s_acc
            
            self.gyro_x=(self.GyX-self.gyro_x_cal)/self.s_gyro
            self.gyro_y=(self.GyY-self.gyro_y_cal)/self.s_gyro
            self.gyro_z=(self.GyZ-self.gyro_z_cal)/self.s_gyro

            # Roll and pitch from accelerometer
            self.acc_roll=180.*m.atan2(self.acc_y, self.acc_z)/m.pi
            self.acc_pitch=-180.*m.atan2(self.acc_x, self.acc_z)/m.pi
            print "acc_roll and acc_pitch :", self.acc_roll, self.acc_pitch

            # Roll, pitch and yaw from gyroscope
            self.gyro_roll+=180.*dt*self.gyro_x/m.pi
            self.gyro_pitch+=180.*dt*self.gyro_y/m.pi
            self.gyro_yaw+=180.*dt*self.gyro_z/m.pi

            self.roll=self.alpha*(self.roll+180.*self.gyro_x*dt/m.pi)+(1-self.alpha)*self.acc_roll
            self.pitch=self.alpha*(self.pitch+180.*self.gyro_y*dt/m.pi)+(1-self.alpha)*self.acc_pitch

            self.send_computed_data()

    def calibrate(self):
        print("CALIBRATING===========================")
        self.acc_x_cal=sum(self.data_dict["AcX"][0:self.N_sample])/self.N_sample
        self.acc_y_cal=sum(self.data_dict["AcY"][0:self.N_sample])/self.N_sample
        self.acc_z_cal=0

        self.gyro_x_cal=sum(self.data_dict["GyX"][0:self.N_sample])/self.N_sample
        self.gyro_y_cal=sum(self.data_dict["GyY"][0:self.N_sample])/self.N_sample
        self.gyro_z_cal=sum(self.data_dict["GyZ"][0:self.N_sample])/self.N_sample

        self.calibrated=True
        print("CALIBRATED")

    def send_computed_data(self):
        print "gyro : ", self.gyro_x, self.gyro_y, self.gyro_z
        print "rpy : ", self.roll, self.pitch, self.gyro_yaw

        self.send(0x17, self.acc_x, self.acc_y, self.acc_z)
        self.send(0x18, self.gyro_x, self.gyro_y, self.gyro_z)
        self.send(0x19, self.roll, self.pitch, self.gyro_yaw)

    def send(self, cmd, d1, d2, d3):
        message[0] = cmd
        message[1] = 6
        message[2] = d1
        message[3] = d2
        message[4] = d3
        conn.send(fmt2.pack(*message))

        conn.send("Hello_world")


def shutdown(signum=None, frame=None):
    print "shutdown", signum, frame
    s.close()
    sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('teleport')
    signal(SIGINT, shutdown)
    signal(SIGTERM, shutdown)

    data_proc=DataProcessor()

    rospy.Subscriber("/msg_SPI", msg_SPI, data_proc.on_msgSPI)
    rospy.spin()
