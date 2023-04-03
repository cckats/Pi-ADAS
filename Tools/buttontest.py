import time
from gpiozero import Button

buttonPin =16

button = Button(buttonPin)



while True:
	print("\n\n\n\n")
	button.wait_for_press()
	print("button pressed")
