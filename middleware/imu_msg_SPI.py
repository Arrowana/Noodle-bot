#!/usr/bin/python
import rospy
from robot.msg import *

from DataProcessor import DataProcessor
from DataTCPSender import DataTCPSender

publish_imu = False

class ImuFirmware:
    def __init__(self, dataProcessor):
        self.dataProcessor=dataProcessor

    def on_msgSPI(self, msg):
        self.AcX=msg.imu_acc_x
        self.AcY=msg.imu_acc_y
        self.AcZ=msg.imu_acc_z

        self.GyX=msg.imu_roll
        self.GyY=msg.imu_pitch
        self.GyZ=msg.imu_yaw

        self.dataProcessor.data_dict["AcX"].append(self.AcX)
        self.dataProcessor.data_dict["AcY"].append(self.AcY)
        self.dataProcessor.data_dict["AcZ"].append(self.AcZ)

        self.dataProcessor.data_dict["GyX"].append(self.GyX)
        self.dataProcessor.data_dict["GyY"].append(self.GyY)
        self.dataProcessor.data_dict["GyZ"].append(self.GyZ)

        self.dataProcessor.compute()

if __name__ == '__main__':
    rospy.init_node('compute_imu')
    data_queue = []

    data_proc=DataProcessor(data_queue)
    imu_rec=ImuFirmware(data_proc)

    data_sender = DataTCPSender(data_queue)
    data_sender.daemon=True
    data_sender.start()

    rospy.Subscriber("/msg_SPI", msg_SPI, imu_rec.on_msgSPI)
    print "Initialization passed"
    rospy.spin()
