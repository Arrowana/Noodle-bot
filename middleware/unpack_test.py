import struct

message=[0]*5

fmt = struct.Struct('2B 3f')

message[0] = 0x02
message[1] = 6
message[2] = 1.2
message[3] = 1.2
message[4] = 1.2

a=fmt.pack(*message)

print fmt.unpack(a)
