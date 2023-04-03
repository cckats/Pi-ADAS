# Pi-ADAS

The purpose of this work is to develop open-source software embedded in an 
affordable Raspberry Pi System, which can be installed in existing old vehicles and 
with the use of computer vision and other sensors have the ability to recognize and 
warn of impending traffic accidents or other dangerous situations, with the goal of the 
driver taking action and preventing them from escalating.

This program takes a feed from the camera or videofile and does lane detection using houghlines and 
object collission, car departure and run red light detection using bonding boxes from mobile_net_v3_small 
with TensorFlow Lite 

Details of Software and Neural Network Model for Object Detection:

- Language: Python
- Framework: TensorFlow Lite
- Network: SSD MobileNet-V3_small
- Training Dataset: COCO Dataset

## Build instructions

### Requirements:

#### Hardware:
- Raspberry Pi 4 2gb or grater
- Usb or Pi Camera (logitech c920 recomended)
- Car Speed recognition (OBDII GPS Module can also be used (code not included))
- Turn signal recognition (Hall effect sensor)
- Status Lights (Blinkt! used)
- Power Supply (Usb Car Charger 15W+ recomended)
- Warning Buzzer (active buzzer recomended for clearer sound. Output jack 3.5 can also be used)

#### Software:
- [Python 3.10](https://www.python.org) (bugs in 3.11+)
- opencv_python 4.6 or newer
- tensorflow 2.11 (not tested in later versions)
- pygame 2.1.2 or newer
- obd 0.7.1 or newer
- pyserial 3.5 or newer

### Install the dependencies:
```
python -m pip install -r requirements.txt 
```
#### GPIO Pinout: 

Turn signal recognition (Hall effect sensor): 
Power 5V: Pin 2, Ground: Pin 20, Data: Pin 22 (GPIO 25) 

Status Lights (Blinkt!): 
Power 5V: Pin 4, Ground: Pin 6, Data: Pins 16,18 (GPIO 23,24) 

Warning Buzzer: 
Ground: Pin 39, Data: Pin 33 (GPIO 13) 

Data pins can be adjusted inside the Pi-ADAS.py file


### Connecting with OBDII
Via SSH Shell from Phone:
```
Bluetoothctl 
scan on 
pair XX:XX:XX:XX:XX:XX (mac address του OBD2) 
trust XX:XX:XX:XX:XX:XX
exit
```

### Automatic Start with power On

```
xterm -hold -e 'sudo rfcomm bind rfcomm9 XX:XX:XX:XX:XX:XX && 
cd /home/pi/Desktop/Py-ADAS/ && 
/usr/bin/python /home/pi/Desktop/Py-ADAS/PI-ADAS.py'
```
Where XX:XX:XX:XX:XX:XX is mac address of OBDII

### Configurable Parameters 

The Pi-ADAS.py file includes at the start all the debug states and configurable parameters of the system


### More detailed explanation of the code and the proposed layout and usage of the system can be found here (in greek!):
[Advanced driver-assistance system using Raspberry Pi, Python and OpenCV](https://polynoe.lib.uniwa.gr/xmlui/handle/11400/4071)

#### Enjoy!! :)
