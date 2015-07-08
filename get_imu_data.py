#!/usr/bin/env python
import rospy

import serial

from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
import random
from threading import Thread, Event
import time
import math as m

from tf.transformations import quaternion_from_euler


SERIALPORT="/dev/ttyUSB0"

class DataContainer(Thread):
    def __init__(self):
        Thread.__init__(self)
        
        maxData=0
        self.ser = serial.Serial(SERIALPORT, 115200)
        self.data_dict={}

        self.calibrated=False

        self.s_acc=1.
        self.s_gyro=1./16.4 #1./16.4 #1./131. # sensivity
        self.N_sample=150
        self.alpha=0.98

        self.lp_alpha=0.99 # alpha=Tf/(Tf+Ts) Tf time constant Ts sampling period. Ts=0.020
        self.low_pass=False

        self.imu_raw_pub = rospy.Publisher('imu_raw', Imu, queue_size=10)
        self.imu_filtered_pub = rospy.Publisher('imu_filtered', Imu, queue_size=10)

    def run(self):   
        while True:
            #print("Data received")
            data_raw=self.ser.readline()
            #print(data_raw)
            data=data_raw.decode("utf-8")
            self.process(str(data))
            #time.sleep(1)

    def process(self, data):
        if "data:" in data:
            try:
                header, data_name=data.split(":")
                data_name,=data_name.splitlines()
            except:
                print("Unexpected format for data_name")
                return

            if data_name == "compute" and "AcX" in self.data_dict:
                print "compute data"
                self.compute()
            else:
                try:
                    # Read value associated to data_name
                    value_raw=self.ser.readline()
                    value=float(value_raw.decode("utf-8"))

                    # Store in dictionnary and add key if doesn't exist
                    if data_name in self.data_dict:
                        self.data_dict[data_name].append(value)
                    else:
                        self.data_dict[data_name]=[value]
                except:
                    print("Unexpected format of data")

    def publish_ros(self):
        imu_msg=Imu()
        imu_msg.header.stamp=rospy.Time.now()
        imu_msg.header.frame_id="map"
        quat=quaternion_from_euler(-m.pi*self.data_dict["acc_roll"][-1]/180, m.pi*self.data_dict["acc_pitch"][-1]/180, m.pi*self.data_dict["gyro_yaw"][-1]/180)
        imu_msg.orientation.x=quat[0]
        imu_msg.orientation.y=quat[1]
        imu_msg.orientation.z=quat[2]
        imu_msg.orientation.w=quat[3]

        self.imu_raw_pub.publish(imu_msg)

        quat=quaternion_from_euler(-m.pi*self.data_dict["roll"][-1]/180, m.pi*self.data_dict["pitch"][-1]/180, m.pi*self.data_dict["gyro_yaw"][-1]/180)
        imu_msg.orientation.x=quat[0]
        imu_msg.orientation.y=quat[1]
        imu_msg.orientation.z=quat[2]
        imu_msg.orientation.w=quat[3]

        self.imu_filtered_pub.publish(imu_msg)
            
    def compute(self):
        if self.calibrated==False and len(self.data_dict["AcX"])<self.N_sample+1:
            print "Remaining values needed for calibration:", self.N_sample-len(self.data_dict["AcX"])

        if self.calibrated==False and len(self.data_dict["AcX"])>self.N_sample+1:
            self.calibrate()

            self.data_dict["acc_x"]=[0]
            self.data_dict["acc_y"]=[0]
            self.data_dict["acc_z"]=[0]

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
            #If time is sent use dt received
            if "dt" in self.data_dict:
                print "dt:", self.data_dict["dt"][-1]
                dt=self.data_dict["dt"][-1]*0.001

            acc_x=(self.data_dict["AcX"][-1]-self.acc_x_cal)*self.s_acc
            if self.low_pass:
                acc_x=self.lp_alpha*self.data_dict["acc_x"][-1]+(1-self.lp_alpha)*acc_x
            self.data_dict["acc_x"].append(acc_x)

            acc_y=(self.data_dict["AcY"][-1]-self.acc_y_cal)*self.s_acc
            if self.low_pass:
                acc_y=self.lp_alpha*self.data_dict["acc_y"][-1]+(1-self.lp_alpha)*acc_y
            self.data_dict["acc_y"].append(acc_y)

            acc_z=(self.data_dict["AcZ"][-1]-self.acc_z_cal)*self.s_acc
            if self.low_pass:
                acc_z=self.lp_alpha*self.data_dict["acc_z"][-1]+(1-self.lp_alpha)*acc_z
            self.data_dict["acc_z"].append(acc_z)
            
            gyro_x=(self.data_dict["GyX"][-1]-self.gyro_x_cal)*self.s_gyro
            self.data_dict["gyro_x"].append(gyro_x)
            gyro_y=-(self.data_dict["GyY"][-1]-self.gyro_y_cal)*self.s_gyro
            self.data_dict["gyro_y"].append(gyro_y)
            gyro_z=-(self.data_dict["GyZ"][-1]-self.gyro_z_cal)*self.s_gyro
            self.data_dict["gyro_z"].append(gyro_z)

            # Roll and pitch from accelerometer
            acc_roll=180.*m.atan2(acc_y, acc_z)/m.pi
            self.data_dict["acc_roll"].append(acc_roll)

            acc_pitch=180.*m.atan2(acc_x,acc_z)/m.pi
            self.data_dict["acc_pitch"].append(acc_pitch)

            # Roll, pitch and yaw from gyroscope
            gyro_roll=self.data_dict["gyro_roll"][-1]+dt*gyro_x
            self.data_dict["gyro_roll"].append(gyro_roll)
            
            gyro_pitch=self.data_dict["gyro_pitch"][-1]+dt*gyro_y
            self.data_dict["gyro_pitch"].append(gyro_pitch)

            gyro_yaw=self.data_dict["gyro_yaw"][-1]+dt*gyro_z
            self.data_dict["gyro_yaw"].append(gyro_yaw) # To have same orientation has DMP add minus
            print "gyro_yaw:", gyro_yaw
            if "yaw_dmp" in self.data_dict:
                print "yaw_dmp:", self.data_dict["yaw_dmp"][-1]

            last_roll=self.data_dict["roll"][-1]
            roll=self.alpha*(last_roll+gyro_x*dt)+(1-self.alpha)*acc_roll
            self.data_dict["roll"].append(roll)

            last_pitch=self.data_dict["pitch"][-1]
            pitch=self.alpha*(last_pitch+gyro_y*dt)+(1-self.alpha)*acc_pitch
            self.data_dict["pitch"].append(pitch)

            sample_size=100
            if(len(self.data_dict["GyX"])==sample_size):
                drift=abs(self.data_dict["GyX"][-1]-self.data_dict["GyX"][0])/(1000*dt)
                print("drift : "+str(drift))

            self.publish_ros()
            #self.clean_data_dict()

    def clean_data_dict(self):
        for key in self.data_dict.keys():
            length=len(self.data_dict[key])
            if(length>400):
                del self.data_dict[key][:150]

            print(length)

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

if __name__ == '__main__':
    rospy.init_node('IMU_publisher', anonymous=True)

    #Create data container
    try:
        data=DataContainer()
        data.daemon=True
        data.start()
    except(KeyboardInterrupt, SystemExit):
        print('\n! Received keyboard interrupt, quitting threads.\n')

    rospy.spin()

   


