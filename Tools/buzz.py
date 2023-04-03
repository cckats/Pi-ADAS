"""from gpiozero import Buzzer
from time import sleep

buzzer = Buzzer(13)

while True:
    buzzer.on()
    sleep(0.0001)
    buzzer.off()
    sleep(0.001)

#while True:
 #   buzzer.beep()



import sys
import RPi.GPIO as GPIO
import time
#Trigger PIN (pin where positive cable is connected) is defined in a variable for better management. Be aware to use GPIO BCM naming. In my cabling it is number 14. It is also set to be an output pin:

#GPIO.cleanup()

triggerPIN = 13
GPIO.setmode(GPIO.BCM)
GPIO.setup(triggerPIN,GPIO.OUT)
#A new PWM instance is defined (and named buzzer, but you can use whatever name you prefer). Note that in instance definition also frequency is specified (1KHz).

#The buzzer is then started with a dutycycle of 10 (value between 0 and 100):

buzzer = GPIO.PWM(triggerPIN, 800) # Set frequency to 1 Khz
buzzer.start(10) # Set dutycycle to 10

time.sleep(4)
GPIO.cleanup()
sys.exit()
"""



A0=1
B0=31
C1=33
CS1=35
D1=37
DS1=39
E1=41
F1=44
FS1=46
G1=49
GS1=52
A1=55
AS1=58
B1=62
C2=65
CS2=69
D2=73
DS2=78
E2=82
F2=87
FS2=93
G2=98
GS2=104
A2=110
AS2=117
B2=123
C3=131
CS3=139
D3=147
DS3=156
E3=165
F3=175
FS3=185
G3=196
GS3=208
A3=220
AS3=233
B3=247
C4=262
CS4=277
D4=294
DS4=311
E4=330
F4=349
FS4=370
G4=392
GS4=415
A4=440
AS4=466
B4=494
C5=523
CS5=554
D5=587
DS5=622
E5=659
F5=698
FS5=740
G5=784
GS5=831
A5=880
AS5=932
B5=988
C6=1047
CS6=1109
D6=1175
DS6=1245
E6=1319
F6=1397
FS6=1480
G6=1568
GS6=1661
A6=1760
AS6=1865
B6=1976
C7=2093
CS7=2217
D7=2349
DS7=2489
E7=2637
F7=2794
FS7=2960
G7=3136
GS7=3322
A7=3520
AS7=3729
B7=3951
C8=4186
CS8=4435
D8=4699
DS8=4978

song = [
  G4,
  E4, F4, G4, G4, G4,
  A4, B4, C5, C5, C5,
  E4, F4, G4, G4, G4,
  A4, G4, F4, F4,
  E4, G4, C4, E4,
  D4, F4, B3,
  C4
]

beep = [ 1,440]
beepbeat = [ 4 ,4 ]


beat = [
  8,
  8, 8, 4, 4, 4,
  8, 8, 4, 4, 4,
  8, 8, 4, 4, 4,
  8, 8, 4, 2,
  4, 4, 4, 4,
  4, 2, 4,
  
  1
]

import RPi.GPIO as GPIO
import time
from threading import Thread

buzzerPin = 13

activeBuzzer=True
playing=False
if activeBuzzer:
    
    #global buzzerPin
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(buzzerPin, GPIO.OUT, initial=1) 
    #GPIO.setwarnings(False)
    GPIO.output(buzzerPin,1)
    # self.Buzz = GPIO.PWM(buzzerPin, 440) 
    #self.Buzz.start(50) 

    def playbeep(intensity):
        global playing
        playing=True
        GPIO.output(buzzerPin,0)
        time.sleep(0.2/intensity)
        GPIO.output(buzzerPin,1)
        time.sleep(0.2/intensity)
        playing=False

    def play(intensity=1):
        global playing
        if intensity > 7 or intensity < 1 :
            return
        if not playing:
            print("play") 
            Thread(target=playbeep, args=(intensity,)).start()
        else:
          print("already playing") 
else:
    playing=False
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(buzzerPin, GPIO.OUT) 
    GPIO.setwarnings(False)

    global Buzz 
    Buzz = GPIO.PWM(buzzerPin, 440) 

    def playsong():
	    global playing
	    Buzz.start(25) 
	    for i in range(1, len(song)): 
		    Buzz.ChangeFrequency(song[i]) 
		    time.sleep(beat[i]*0.13) 
	    playing=False

    def playbeep(intensity):
	    global playing
	    playing=True
	    Buzz.start(6)
	    for i in range(1, len(beep)): 
		    Buzz.ChangeFrequency(beep[i]*intensity) 
		    time.sleep(beepbeat[i]/intensity*0.13) 
	    playing=False
	    Buzz.stop()

    def play(intensity=1):
	    if intensity > 5 or intensity < 1 :
		    intensity=1
	    global playing
	    if not playing:
		    Thread(target=playsong, args=(intensity,)).start()
	    else:
		    print("already playing") 

while True:
	#print("test")
	play(1)
	time.sleep(0.1)
