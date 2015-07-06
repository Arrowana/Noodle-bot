#!/usr/bin/env python
import rospy

import serial

from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
from threading import Thread, Event
import time
import math as m

from tf.transformations import quaternion_from_euler


SERIALPORT="/dev/ttyUSB0"

class DataContainer(Thread):
    def __init__(self, axes):
        Thread.__init__(self)
        
        maxData=0
        self.ser = serial.Serial(SERIALPORT, 115200)
        self.data_dict={}
        self.axes=axes

        self.calibrated=False

        self.s_acc=1.
        self.s_gyro=1./16.4 #1./131. # sensivity
        self.N_sample=150
        self.alpha=0.98

        self.lp_alpha=0.99 # alpha=Tf/(Tf+Ts) Tf time constant Ts sampling period. Ts=0.020
        self.low_pass=False

        self.imu_raw_pub = rospy.Publisher('imu_raw', Imu, queue_size=10)
        self.imu_filtered_pub = rospy.Publisher('imu_filtered', Imu, queue_size=10)

    def update(self, frameNum):
        # For each axis iterate through each line to set the data
        for axe in self.axes:
            y_max_list=[]
            isData=False

            for line in axe.get_lines():
                data_name=line.get_label()

                if data_name in self.data_dict:
                    if self.data_dict[data_name]:
                        y_max=self.modify(line, data_name)
                        y_max_list.append(y_max)

                        isData=True
            if isData:
                # Set ylim to the max of y data in the graph
                global_max=max(y_max_list)
                axe.set_ylim(-global_max, global_max) 

    def modify(self, line, data_name):
        #Plot only N_values
        length=100
        length_data=len(self.data_dict[data_name])

        if(length_data<length):
            length=length_data

        line.set_data(range(length), self.data_dict[data_name][-length:])
        line.axes.set_xlim(0, length)
        y_max=max([abs(x) for x in self.data_dict[data_name]])
        return y_max

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

    def pop_data(self, data_list):
        data_list.append(random.randint(0,100))

if __name__ == '__main__':
    rospy.init_node('IMU_publisher', anonymous=True)

    # Set up animation
    # please keep in mind that the label of a line is used for ploting data
    fig = plt.figure(0)
    ax_acc=plt.subplot(331)
    plt.title("acc calibrated")
    ax_acc.plot([],[],label="acc_x")
    ax_acc.plot([],[],label="acc_y")
    ax_acc.plot([],[],label="acc_z")
    
    ax_gyro=plt.subplot(332)
    plt.title("gyro calibrated")
    ax_gyro.plot([],[], label="gyro_x")
    ax_gyro.plot([],[], label="gyro_y")
    ax_gyro.plot([],[], label="gyro_z")

    ax_roll_comb=plt.subplot(333)
    plt.title("acc_roll, gyro_roll and roll")
    ax_roll_comb.plot([],[], label="acc_roll")
    ax_roll_comb.plot([],[], label="gyro_roll")
    ax_roll_comb.plot([],[], label="roll")

    ax_pitch_comb=plt.subplot(334)
    plt.title("acc_pitch, gyro_pitch and pitch")
    ax_pitch_comb.plot([],[], label="acc_pitch")
    ax_pitch_comb.plot([],[], label="gyro_pitch")
    ax_pitch_comb.plot([],[], label="pitch")

    ax_yaw=plt.subplot(335)
    plt.title("gyro_yaw")
    ax_yaw.plot([],[], label="gyro_yaw")

    ax_roll_all=plt.subplot(337)
    plt.title("roll and roll_dmp")
    ax_roll_all.plot([],[], label="roll")
    ax_roll_all.plot([],[], label="roll_dmp")

    ax_pitch_all=plt.subplot(338)
    plt.title("pitch and pitch_dmp")
    ax_pitch_all.plot([],[], label="pitch")
    ax_pitch_all.plot([],[], label="pitch_dmp")

    ax_yaw_all=plt.subplot(339)
    plt.title("gyro_yaw and yaw_dmp")
    ax_yaw_all.plot([],[], label="gyro_yaw")
    ax_yaw_all.plot([],[], label="yaw_dmp")
    
    axes=[ax_acc, ax_gyro, ax_roll_comb, ax_pitch_comb, ax_yaw, ax_roll_all, ax_pitch_all, ax_yaw_all]

    #Create data container
    try:
        data=DataContainer(axes)
        data.daemon=True
        data.start()
    except(KeyboardInterrupt, SystemExit):
        print('\n! Received keyboard interrupt, quitting threads.\n')

    anim = animation.FuncAnimation(fig, data.update, interval=50)
    plt.show()

   


