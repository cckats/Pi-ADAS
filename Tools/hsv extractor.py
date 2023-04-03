import cv2
import numpy as np

def nothing(x):
    pass


skip = 0000

# Create trackbars for color change
# Hue is from 0-179 for Opencv
cv2.namedWindow('image')
cv2.createTrackbar('HMin', 'image', 0, 179, nothing)
cv2.createTrackbar('SMin', 'image', 0, 255, nothing)
cv2.createTrackbar('VMin', 'image', 0, 255, nothing)
cv2.createTrackbar('HMax', 'image', 0, 179, nothing)
cv2.createTrackbar('SMax', 'image', 0, 255, nothing)
cv2.createTrackbar('VMax', 'image', 0, 255, nothing)
frameCount=0

# Set default value for Max HSV trackbars
cv2.setTrackbarPos('HMin', 'image', 92)
cv2.setTrackbarPos('SMin', 'image', 79)
cv2.setTrackbarPos('VMin', 'image', 207)
cv2.setTrackbarPos('HMax', 'image', 130)
cv2.setTrackbarPos('SMax', 'image', 255)
cv2.setTrackbarPos('VMax', 'image', 255)

# Initialize HSV min/max values
hMin = sMin = vMin = hMax = sMax = vMax = 0
phMin = psMin = pvMin = phMax = psMax = pvMax = 0
# Load image
cap = cv2.VideoCapture("lights tp.mp4")
while(cap.isOpened()):
    _, image = cap.read()
    # Create a window
    frameCount=frameCount+1

    imageor=image
    
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    if frameCount >= skip:
        while(1):
            
            # Get current positions of all trackbars
            hMin = cv2.getTrackbarPos('HMin', 'image')
            sMin = cv2.getTrackbarPos('SMin', 'image')
            vMin = cv2.getTrackbarPos('VMin', 'image')
            hMax = cv2.getTrackbarPos('HMax', 'image')
            sMax = cv2.getTrackbarPos('SMax', 'image')
            vMax = cv2.getTrackbarPos('VMax', 'image')

            # Set minimum and maximum HSV values to display
            lower = np.array([hMin, sMin, vMin])
            upper = np.array([hMax, sMax, vMax])

            # Convert to HSV format and color threshold
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower, upper)
            result = cv2.bitwise_and(image, image, mask=mask)

            # Print if there is a change in HSV value
            if((phMin != hMin) | (psMin != sMin) | (pvMin != vMin) | (phMax != hMax) | (psMax != sMax) | (pvMax != vMax) ):
                print("(hMin = %d , sMin = %d, vMin = %d), (hMax = %d , sMax = %d, vMax = %d)" % (hMin , sMin , vMin, hMax, sMax , vMax))
                phMin = hMin
                psMin = sMin
                pvMin = vMin
                phMax = hMax
                psMax = sMax
                pvMax = vMax

            # Display result image
            result = cv2.cvtColor(result, cv2.COLOR_BGR2RGB)
            frameall = np.hstack((imageor, result))
            cv2.putText(image, 'Frame:{0}'.format(frameCount),(30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2, cv2.LINE_AA)
            cv2.imshow('image', frameall)
            #imageor
            #cv2.imshow('image', imageor)
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break

cv2.destroyAllWindows()