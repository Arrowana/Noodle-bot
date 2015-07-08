#!/usr/bin/env python
import rospy

from DataProcessor import DataProcessor
from ImuSerialReceiver import ImuSerialReceiver
from DataTCPSender import DataTCPSender

import serial
import time

SERIALPORT="/dev/ttyUSB0"

if __name__ == '__main__':
    rospy.init_node('IMU_publisher', anonymous=True)

    data_queue=[]

    dataProcessor=DataProcessor(data_queue)

    #Create data container
    serialReceiver=ImuSerialReceiver(SERIALPORT, dataProcessor)
    serialReceiver.daemon=True
    serialReceiver.start()

    data_sender = DataTCPSender(data_queue)
    data_sender.daemon=True
    data_sender.start()

    rospy.spin()

   


