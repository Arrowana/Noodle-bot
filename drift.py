#!/usr/bin/env python
import rospy
import serial
import random
from threading import Thread, Event
import time
import math as m
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import signal
import sys

SERIALPORT="/dev/ttyUSB0"

class DataContainer(Thread):
	def __init__(self, line):
		Thread.__init__(self)
		
		self.line=line

		self.ser = serial.Serial(SERIALPORT, 115200)
		self.data_dict={}

		self.calibrated=False

		self.s_acc=1
		self.s_gyro=4*131*180/m.pi # sensivity+ deg to rad
		self.N_sample=500
		self.alpha=0.98

		self.start_time=rospy.Time.now()
		self.last_time=self.start_time

		self.recorded_data=["GyX","GyY","GyZ"]

	def run(self):   
		while True:
			#print("Data received")
			data_raw=self.ser.readline()
			#print(data_raw)
			data=data_raw.decode("utf-8")
			self.process(str(data))

	def process(self, data):
		if "data:" in data:
			try:
				header, data_name=data.split(":")
				data_name,=data_name.splitlines()
				# Read value associated to data_name
				value_raw=self.ser.readline()
				value=float(value_raw.decode("utf-8"))
			except:
				print("Unexpected format of data")
				return

			if data_name in self.recorded_data:
				# Store in dictionnary and add key if doesn't exist
				if data_name in self.data_dict:
					self.data_dict[data_name].append(value)
				else:
					self.data_dict[data_name]=[value]

			if "GyX" in self.data_dict:
				self.compute()

	def update(self, frameNum):
		data_name=self.line.get_label()
		if data_name in self.data_dict:
			self.modify(self.line, data_name)

	def modify(self, line, data_name):
		length=len(self.data_dict[data_name])
		line.set_data(range(length), self.data_dict[data_name][:length])
		line.axes.set_xlim(0, length)

		y_max=max([abs(x) for x in self.data_dict[data_name]])
		line.axes.set_ylim(-y_max, y_max)
			
	def compute(self):
		if self.calibrated==False and len(self.data_dict["GyX"])>self.N_sample+1:
			self.calibrate()

			self.data_dict["gyro_x"]=[]
			self.data_dict["gyro_y"]=[]
			self.data_dict["gyro_z"]=[]

			self.data_dict["gyro_roll"]=[0]
			self.data_dict["gyro_pitch"]=[0]
			self.data_dict["gyro_yaw"]=[0]

		if self.calibrated==True:
			dt=0.02
			
			gyro_x=(self.data_dict["GyX"][-1]-self.gyro_x_cal)/self.s_gyro
			self.data_dict["gyro_x"].append(gyro_x)
			gyro_y=(self.data_dict["GyY"][-1]-self.gyro_y_cal)/self.s_gyro
			self.data_dict["gyro_y"].append(gyro_y)
			gyro_z=(self.data_dict["GyZ"][-1]-self.gyro_z_cal)/self.s_gyro
			self.data_dict["gyro_z"].append(gyro_z)

			# Roll, pitch and yaw from gyroscope
			gyro_roll=self.data_dict["gyro_roll"][-1]+180.*dt*gyro_x/m.pi
			self.data_dict["gyro_roll"].append(gyro_roll)
			
			gyro_pitch=self.data_dict["gyro_pitch"][-1]+180.*dt*gyro_y/m.pi
			self.data_dict["gyro_pitch"].append(gyro_pitch)

			gyro_yaw=self.data_dict["gyro_yaw"][-1]+180.*dt*gyro_z/m.pi
			self.data_dict["gyro_yaw"].append(gyro_yaw)

			if (rospy.Time.now()-self.last_time).to_sec()>1.:
				time_elapsed=(rospy.Time.now()-self.start_time).to_sec()
				print("================================")
				print("Time elapsed :"+str(time_elapsed))
				print("gyro_yaw: "+str(gyro_yaw))
				print("drift in degrees: "+str(gyro_yaw-self.data_dict["gyro_yaw"][1]))
				self.last_time=rospy.Time.now()

	def calibrate(self):
		print("CALIBRATING===========================")
		self.gyro_x_cal=sum(self.data_dict["GyX"][0:self.N_sample])/self.N_sample
		self.gyro_y_cal=sum(self.data_dict["GyY"][0:self.N_sample])/self.N_sample
		self.gyro_z_cal=sum(self.data_dict["GyZ"][0:self.N_sample])/self.N_sample

		self.calibrated=True
		print("CALIBRATED")

if __name__ == '__main__':
	rospy.init_node('drift', anonymous=True)
	print("Node initialized")

	fig = plt.figure(0)
	ax1=plt.subplot(111)
	plt.title("gyro_yaw")
	line_yaw, = ax1.plot([],[],label="gyro_yaw")

	data=DataContainer(line_yaw)
	data.daemon=True
	data.start()

	anim = animation.FuncAnimation(fig, data.update, interval=50)
	plt.show()
   


