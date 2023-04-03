
import RPi.GPIO as GPIO
import time

tsign_left=False
tsign_right=False

turnPin = 25

class hallSens:
    
    def __init__(self):
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(turnPin, GPIO.IN)
    
    def setTurn(self):
        global tsign_left,tsign_right
        if GPIO.input(turnPin):
            tsign_left = True
            tsign_right = True
        else:
            tsign_left = False
            tsign_right = False
        
hall=hallSens()
while True:
    hall.setTurn()
    print(tsign_left,tsign_right)
    time.sleep(0.1)
    
