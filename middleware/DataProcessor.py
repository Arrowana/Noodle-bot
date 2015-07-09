#!/usr/bin/python
import rospy

from robot.msg import *
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler

import math as m

class DataProcessor:          
	def __init__(self, data_queue):
		self.calibrated=False
		self.data_dict={}
		self.setup_dict()
		self.data_queue=data_queue

		self.last_update = rospy.Time.now()

		self.s_acc=1
		self.s_gyro=1/131. # sensivity+ deg to rad
		self.N_sample=100 #200
		self.alpha=0.98

		self.lp_alpha=0.99 
		self.low_pass=False

		self.imu_pub = rospy.Publisher('imu', Imu, queue_size=10)

	def setup_dict(self):
		self.data_dict["AcX"]=[]
		self.data_dict["AcY"]=[]
		self.data_dict["AcZ"]=[]

		self.data_dict["GyX"]=[]
		self.data_dict["GyY"]=[]
		self.data_dict["GyZ"]=[]

	def compute(self):
		if self.calibrated==False and len(self.data_dict["AcX"])<self.N_sample+1:
			print "Remaining values needed for calibration:", self.N_sample-len(self.data_dict["AcX"])

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

			self.grab_raw()

			acc_x_nf=(self.AcX-self.acc_x_cal)*self.s_acc
			if self.low_pass:
				self.acc_x=self.lp_alpha*self.acc_x+(1-self.lp_alpha)*acc_x_nf
			else:
				self.acc_x=acc_x_nf

			acc_y_nf=(self.AcY-self.acc_y_cal)*self.s_acc
			if self.low_pass:
				self.acc_y=self.lp_alpha*self.acc_y+(1-self.lp_alpha)*acc_y_nf
			else:
				self.acc_y=acc_y_nf

			acc_z_nf=(self.AcZ-self.acc_z_cal)*self.s_acc
			if self.low_pass:
				self.acc_z=self.lp_alpha*self.acc_z+(1-self.lp_alpha)*acc_z_nf
			else:
				self.acc_z=acc_z_nf
			
			self.gyro_x=(self.GyX-self.gyro_x_cal)*self.s_gyro
			self.gyro_y=(self.GyY-self.gyro_y_cal)*self.s_gyro
			self.gyro_z=(self.GyZ-self.gyro_z_cal)*self.s_gyro

			# Roll and pitch from accelerometer
			self.acc_roll=180.*m.atan2(self.acc_y, self.acc_z)/m.pi
			self.acc_pitch=-180.*m.atan2(self.acc_x, self.acc_z)/m.pi

			# Roll, pitch and yaw from gyroscope
			self.gyro_roll+=dt*self.gyro_x
			self.gyro_pitch+=dt*self.gyro_y
			self.gyro_yaw+=dt*self.gyro_z
			# Keep gyro_yaw within -180;180
			if self.gyro_yaw > 185:
				self.gyro_yaw = self.gyro_yaw - 360
			elif self.gyro_yaw < -180:
				self.gyro_yaw = self.gyro_yaw + 360

			self.roll=self.alpha*(self.roll+self.gyro_x*dt)+(1-self.alpha)*self.acc_roll
			self.pitch=self.alpha*(self.pitch+self.gyro_y*dt)+(1-self.alpha)*self.acc_pitch

			self.publish_imu()
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

	def grab_raw(self):
		self.AcX=self.data_dict["AcX"][-1]
		self.data_dict["AcX"]=[]
		self.AcY=self.data_dict["AcY"][-1]
		self.data_dict["AcY"]=[]
		self.AcZ=self.data_dict["AcZ"][-1]
		self.data_dict["AcZ"]=[]
		self.GyX=self.data_dict["GyX"][-1]
		self.data_dict["GyX"]=[]
		self.GyY=self.data_dict["GyY"][-1]
		self.data_dict["GyY"]=[]
		self.GyZ=self.data_dict["GyZ"][-1]
		self.data_dict["GyZ"]=[]

	def send_computed_data(self):
		acc_data = {"acc_x":self.acc_x, "acc_y":self.acc_y,"acc_z":self.acc_z}
		acc_rpy_data = {"acc_roll":self.acc_roll,"acc_pitch":self.acc_pitch}
		gyro_data = {"gyro_x":self.gyro_x,"gyro_y":self.gyro_y,"gyro_z":self.gyro_z}
		gyro_rpy_data = {"gyro_roll":self.gyro_roll,"gyro_pitch":self.gyro_pitch}
		rpy_data = {"roll":self.roll,"pitch":self.pitch,"gyro_yaw":self.gyro_yaw}

		isDmp=False

		if all(name in self.data_dict for name in ["roll_dmp", "pitch_dmp", "yaw_dmp"]):
			dmp_data = {"roll_dmp":self.data_dict,"pitch_dmp":self.data_dict,"yaw_dmp":self.data_dict}
			isDmp=True

		# Lower update of values
		if (rospy.Time.now()-self.last_update).to_sec()>0.2:
			print "roll:",self.roll,"pitch:",self.pitch,"gyro_yaw", self.gyro_yaw

			self.send(acc_data)
			self.send(acc_rpy_data)
			self.send(gyro_data)
			self.send(gyro_rpy_data)
			self.send(rpy_data)

			if isDmp:
				self.send(dmp_data)

			self.last_update = rospy.Time.now()

	def send(self, data):
		self.data_queue.append(data)

	def publish_imu(self):
		imu_msg=Imu()
		imu_msg.header.stamp=rospy.Time.now()
		imu_msg.header.frame_id="map"
		quat=quaternion_from_euler(-m.pi*self.roll/180, m.pi*self.pitch/180, m.pi*self.gyro_yaw/180)
		imu_msg.orientation.x=quat[0]
		imu_msg.orientation.y=quat[1]
		imu_msg.orientation.z=quat[2]
		imu_msg.orientation.w=quat[3]

		self.imu_pub.publish(imu_msg)