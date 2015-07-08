#!/usr/bin/env python
import rospy

import serial
from threading import Thread

class ImuSerialReceiver(Thread):
    def __init__(self, serialport, processor):
        Thread.__init__(self)

        self.ser = serial.Serial(serialport, 115200)
        self.data_dict={}

        self.data_processor=processor

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

            if data_name == "compute" and "AcX" in self.data_processor.data_dict:
                self.data_processor.compute()
            else:
                try:
                    # Read value associated to data_name
                    value_raw=self.ser.readline()
                    value=float(value_raw.decode("utf-8"))

                    # Store in dictionnary and add key if doesn't exist
                    if data_name in self.data_processor.data_dict:
                        self.data_processor.data_dict[data_name].append(value)
                    else:
                        self.data_processor.data_dict[data_name]=[value]
                except:
                    print("Unexpected format of data")