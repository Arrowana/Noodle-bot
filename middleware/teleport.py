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
fmt2 = struct.Struct('2B 3f')
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
        self.data_dict["AcX"].append(msg.imu_acc_x)
        self.data_dict["AcY"].append(msg.imu_acc_y)
        self.data_dict["AcZ"].append(msg.imu_acc_z)

        self.data_dict["GyX"].append(msg.imu_roll)
        self.data_dict["GyY"].append(msg.imu_pitch)
        self.data_dict["GyZ"].append(msg.imu_yaw)

        self.compute()

    def compute(self):
        if self.calibrated==False and len(self.data_dict["AcX"])>self.N_sample+1:
            self.calibrate()

            self.data_dict["acc_x"]=[]
            self.data_dict["acc_y"]=[]
            self.data_dict["acc_z"]=[]

            self.data_dict["gyro_x"]=[]
            self.data_dict["gyro_y"]=[]
            self.data_dict["gyro_z"]=[]

            self.data_dict["acc_pitch"]=[]
            self.data_dict["acc_roll"]=[]

            self.data_dict["gyro_roll"]=[0]
            self.data_dict["gyro_pitch"]=[0]
            self.data_dict["gyro_yaw"]=[0]

            self.data_dict["roll"]=[0]
            self.data_dict["pitch"]=[0]

        if self.calibrated==True:
            dt=0.05

            acc_x=(self.data_dict["AcX"][-1]-self.acc_x_cal)/self.s_acc
            if self.low_pass:
                acc_x=self.lp_alpha*self.data_dict["acc_x"][-1]+(1-self.lp_alpha)*acc_x
            self.data_dict["acc_x"].append(acc_x)

            acc_y=(self.data_dict["AcY"][-1]-self.acc_y_cal)/self.s_acc
            if self.low_pass:
                acc_y=self.lp_alpha*self.data_dict["acc_y"][-1]+(1-self.lp_alpha)*acc_y
            self.data_dict["acc_y"].append(acc_y)

            acc_z=(self.data_dict["AcZ"][-1]-self.acc_z_cal)/self.s_acc
            if self.low_pass:
                acc_z=self.lp_alpha*self.data_dict["acc_z"][-1]+(1-self.lp_alpha)*acc_z
            self.data_dict["acc_z"].append(acc_z)
            
            gyro_x=(self.data_dict["GyX"][-1]-self.gyro_x_cal)/self.s_gyro
            self.data_dict["gyro_x"].append(gyro_x)
            gyro_y=(self.data_dict["GyY"][-1]-self.gyro_y_cal)/self.s_gyro
            self.data_dict["gyro_y"].append(gyro_y)
            gyro_z=(self.data_dict["GyZ"][-1]-self.gyro_z_cal)/self.s_gyro
            self.data_dict["gyro_z"].append(gyro_z)

            # Roll and pitch from accelerometer
            acc_roll=180.*m.atan2(acc_y, acc_z)/m.pi
            self.data_dict["acc_roll"].append(acc_roll)

            acc_pitch=-180.*m.atan2(acc_x,acc_z)/m.pi
            self.data_dict["acc_pitch"].append(acc_pitch)

            # Roll, pitch and yaw from gyroscope
            gyro_roll=self.data_dict["gyro_roll"][-1]+180.*dt*gyro_x/m.pi
            self.data_dict["gyro_roll"].append(gyro_roll)
            
            gyro_pitch=self.data_dict["gyro_pitch"][-1]+180.*dt*gyro_y/m.pi
            self.data_dict["gyro_pitch"].append(gyro_pitch)

            gyro_yaw=self.data_dict["gyro_yaw"][-1]+180.*dt*gyro_z/m.pi
            self.data_dict["gyro_yaw"].append(gyro_yaw)

            last_roll=self.data_dict["roll"][-1]
            roll=self.alpha*(last_roll+180.*gyro_x*dt/m.pi)+(1-self.alpha)*acc_roll
            self.data_dict["roll"].append(roll)

            last_pitch=self.data_dict["pitch"][-1]
            pitch=self.alpha*(last_pitch+180.*gyro_y*dt/m.pi)+(1-self.alpha)*acc_pitch
            self.data_dict["pitch"].append(pitch)

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
        print "gyro : ",self.data_dict["gyro_x"][-1], self.data_dict["gyro_y"][-1], self.data_dict["gyro_z"][-1]
        print "rpy : ", self.data_dict["roll"][-1], self.data_dict["pitch"][-1], self.data_dict["gyro_yaw"][-1]

        self.send(0x17, self.data_dict["acc_x"][-1], self.data_dict["acc_y"][-1], self.data_dict["acc_z"][-1])
        #self.send(0x18, self.data_dict["gyro_x"][-1], self.data_dict["gyro_y"][-1], self.data_dict["gyro_z"][-1])
        #self.send(0x19, self.data_dict["roll"][-1], self.data_dict["pitch"][-1], self.data_dict["gyro_yaw"][-1])

    def send(self, cmd, d1, d2, d3):
        message[0] = cmd
        message[1] = 6
        message[2] = d1
        message[3] = d2
        message[4] = d3
        conn.send(fmt.pack(*message))
        print(fmt.pack(*message))


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
