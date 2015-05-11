import serial

# open serial port
ser = serial.Serial(2, 9600)

while(1):
    out=ser.readline()
    print("Received")
    print(out)
ser.close()
