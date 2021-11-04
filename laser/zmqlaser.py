import cv2
import numpy as np
import screeninfo
import time
import zmq
import socket
import time

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('',8888))
s.setblocking(0)

ctx = zmq.Context()
sock = ctx.socket(zmq.PUB)
sock.bind("tcp://*:5109")
time.sleep(5)
cap = cv2.VideoCapture(0) # video capture source camera (Here webcam of laptop)
#screen = screeninfo.get_monitors()[0]
width, height = 1280,800
img = cv2.imread('./checker16x9_1280.png')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret, original_corners = cv2.findChessboardCorners(gray,(15,9), None)
#cv2.waitKey(2000)
sock.send_string("CAL")
time.sleep(1)
ret,frame = cap.read()
ret,frame = cap.read()
ret,frame = cap.read()
ret,frame = cap.read()
ret,frame = cap.read() # let the camera normalize with a few frames
time.sleep(0.3)
sock.send_string("CLEAR")
# return a single frame in variable `frame`
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#invgray=cv2.bitwise_not(gray)
(T, thresh) = cv2.threshold(gray, 0, 255,
	cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)

ret, found_corners = cv2.findChessboardCorners(thresh, (15, 9), None)

h,status = cv2.findHomography(found_corners, original_corners)
# from her we can warp perspective

run=True
mode=0
kernel = np.ones((5, 5), 'uint8')
OldX=0
OldY=0
while(run):
    ret, frame=cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0,0,244])
    upper_red = np.array([195,55,255])
    mask = cv2.inRange(hsv, lower_red, upper_red)
    res = cv2.bitwise_and(frame,frame, mask= mask)
    #erode_img = cv2.erode(res, kernel, iterations=1)
    dilate_img = cv2.dilate(res, kernel, iterations=1)
    im_dst = cv2.warpPerspective(dilate_img, h, (width,height))
    gray = cv2.cvtColor(im_dst, cv2.COLOR_BGR2GRAY)
    contours, hierarchy = cv2.findContours(image=gray, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
    # loop over the contours
    if len(contours) != 0:
        c = max(contours, key = cv2.contourArea)
        M = cv2.moments(c)
        if (M["m00"] > 0):
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        normalizedcX=cX/1280
        normalizedcY=cY/800
        outstring = 'X:' + str(normalizedcX) + 'Y:' + str(normalizedcY)
        #print(outstring)
        try:
            data,address = s.recvfrom(1024)
        except socket.error:
            pass
        else:
            print (outstring)
            sock.send_string(outstring)
    else:
        try:
            data,address=s.recvfrom(1024)
        except socket.error:
            pass


