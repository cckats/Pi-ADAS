import obd
import time
import datetime
import pickle
import numpy as np
#from matplotlib import pyplot as plt




#bluetoothctl         scan on   'pair' mac of obd    'trust' mac of obd           sudo rfcomm bind rfcomm99 mac of obd  
ports = obd.scan_serial()
print(ports) 
obd.logger.setLevel(obd.logging.DEBUG)
#while True:
#connection = obd.OBD(portstr='com3',baudrate=38400, protocol=None,timeout=30,fast=False)
#time.sleep(1)
#	if connection.is_connected():
#		breakbaudrate=38400, protocol=None,baudrate=9600,

connection = obd.OBD(portstr='/dev/rfcomm9',timeout=30,fast=False) # same constructor as 'obd.OBD()'

#connection.watch(obd.commands.SPEED) # keep track of the RPM

#connection.start() # start the async update loop

cmd = obd.commands.SPEED
oldSpeed=0
prevTime = datetime.datetime.now()
time=0
accelLog=[]
avgAccel=0

N=10

avglog=[]
for n in range(0,N-1):
	avglog.append(np.NaN)
 
oldaccel=0

try:#print(connection.status(),protocol_name())
	while True:
		r = connection.query(cmd)
		currTime = datetime.datetime.now()
		time =(currTime-prevTime).microseconds/1000000
		#print(time)
		prevTime=currTime

		accel = (r.value.magnitude - oldSpeed) / time
		oldSpeed = r.value.magnitude
		print(' Speed:', end = '')
		print(r.value.magnitude, end = ' AvgAccel:')


		#oldaccel=(accel+oldaccel)/2
		accelLog.append(accel)
		avgAccel = np.mean(accelLog[:-5])
		avglog.append(np.mean(accelLog[:-5]))
		print(avgAccel)
		#print(np.mean(accelLog[:-5]))

finally:
	connection.close()
	with open('accell','wb') as fp:
		pickle.dump(accelLog,fp)
	if len(accelLog)>0:
		#plt.plot(accelLog)
		
		for n in range(0,N-1):
			avglog.append(np.NaN)
		for num in range(N,len(accelLog)+1):
			avglog.append(np.nanmean(accelLog[num-N:num]))
			
		#plt.plot(avglog)
		#plt.show()
