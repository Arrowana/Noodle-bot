import socket
import json
import time

data = {'message':'hello world!', 'test':123.4}

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('127.0.0.1', 13373))
#s.send(json.dumps(data))
for i in range(10):
	try:
		result = json.loads(s.recv(1024))
		print result
	except ValueError:
		print 'Error loading'

	time.sleep(0.1)
s.close()