blinkt = __import__ (blinktf)
import time,colorsys
"""

class statLED:
	blinkt.set_brightness(1)
	blinkt.clear() 
	turnl=0
	leftlane=0
	carlock=0
	rightlane=0
	light=0
	turnr=0
    
	def __init__(self):
		self.turnl=1
		self.leftlane=1
		self.carlock=1
		self.rightlane=1
		self.light=1
		self.turnr=1
		
		self.show()

		
	def show(self):
		blinkt.clear()class statLED:
	blinkt.set_brightness(1)
	blinkt.clear() 
	turnl=0
	leftlane=0
	carlock=0
	rightlane=0
	light=0
	turnr=0
    
	def __init__(self):
		self.turnl=1
		self.leftlane=1
		self.carlock=1
		self.rightlane=1
		self.light=1
		self.turnr=1
		
		self.show()

		
	def show(self):
		blinkt.clear()
		if self.turnl:
			blinkt.set_pixel(0, 255, 255, 0)
		if self.leftlane:
			blinkt.set_pixel(1, 0, 255, 0)
		if self.carlock:
			blinkt.set_pixel(2, 0, 0, 255)
		if self.rightlane:
			blinkt.set_pixel(3, 0, 255, 0)
			
		if self.light:
			blinkt.set_pixel(5, 255, 0, 0)
			
		if self.turnr:
			blinkt.set_pixel(7, 255, 255, 0)
		blinkt.show()
		
		turnl=0,
		leftlane=0
		carlock=0
		rightlane=0
		light=0
		turnr=0
        
statled = statLED();

statled.turnl=1
statled.show()

time.sleep(10)
#statled.setcolors(1,1,1,1,1,1)


set_brightness(0.1)
clear() 
def setcolors(turnl=0,leftlane=0,carlock=0,rightlane=0,light=0,turnr=0):
	clear()
	if turnl:
		set_pixel(0, 255, 255, 0)
	if leftlane:
		set_pixel(1, 0, 255, 0)
	if carlock:
		set_pixel(2, 0, 0, 255)
	if rightlane:
		set_pixel(3, 0, 255, 0)
		
	if light:
		set_pixel(5, 255, 0, 0)
		
	if turnr:
		set_pixel(7, 255, 255, 0)
	#Thread(target=showcol, args=()).start()\
#while True:
	#show()


while True:
	print("test")
	setcolors(turnl=1,rightlane=1,light=1)
	show()
	#clear()
	time.sleep(0.1)
"""
spacing = 360.0 / 16.0
hue = 0

blinkt.set_brightness(2) 

while True:
	hue = int(time.time() * 100) % 360
	for x in range(8):
		offset = x * spacing
		h = ((hue + offset) % 360) / 360.0
		r, g, b = [int(c * 255) for c in colorsys.hsv_to_rgb(h, 1.0, 1.0)]
		blinkt.set_pixel(x, r, g, b)
	blinkt.show()
	time.sleep(0.001)
	blinkt.clear()

	
