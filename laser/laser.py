import cv2
import numpy as np
import screeninfo
import time

cap = cv2.VideoCapture(0) # video capture source camera (Here webcam of laptop)
screen = screeninfo.get_monitors()[0]
width, height = 1280,800
img = cv2.imread('./checker16x9_1280.png')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret, original_corners = cv2.findChessboardCorners(gray,(15,9), None)
window_name = 'projector'
cv2.namedWindow(window_name, cv2.WND_PROP_FULLSCREEN)
cv2.moveWindow(window_name, screen.x - 1, screen.y - 1)
cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
cv2.imshow('projector',img)
cv2.waitKey(2000)
ret,frame = cap.read() # return a single frame in variable `frame`
ret,frame = cap.read() # return a single frame in variable `frame`
ret,frame = cap.read() # return a single frame in variable `frame`
ret,frame = cap.read() # return a single frame in variable `frame`
ret,frame = cap.read() # return a single frame in variable `frame`

gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#invgray=cv2.bitwise_not(gray)
(T, thresh) = cv2.threshold(gray, 0, 255,
	cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)

ret, found_corners = cv2.findChessboardCorners(thresh, (15, 9), None)
# Let's show off some debug info!
image=cv2.drawChessboardCorners(frame, (15,9), found_corners, ret	)

cv2.imshow('projector',image)
cv2.waitKey(2000)
print("Original Corners")
print(original_corners)
print("New corners")
print(found_corners)

h,status = cv2.findHomography(found_corners, original_corners)
# from her we can warp perspective
print("Homography map")
print(h)
blank = np.zeros((height,width,3), np.uint8)
run=True
mode=0
kernel = np.ones((5, 5), 'uint8')
OldX=0
OldY=0
while(run):
    ret, frame=cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    lower_red = np.array([0,0,200])
    upper_red = np.array([255,110,255])
    
    mask = cv2.inRange(hsv, lower_red, upper_red)
    res = cv2.bitwise_and(frame,frame, mask= mask)
    #erode_img = cv2.erode(res, kernel, iterations=1)
    dilate_img = cv2.dilate(res, kernel, iterations=1)
    im_dst = cv2.warpPerspective(dilate_img, h, (width,height))
    gray = cv2.cvtColor(im_dst, cv2.COLOR_BGR2GRAY)
    
    contours, hierarchy = cv2.findContours(image=gray, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
    if (mode == 0):
        cv2.drawContours(image=blank, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=5, lineType=cv2.LINE_AA)
        cv2.imshow('projector',blank)
        blank = np.zeros((height,width,3), np.uint8)
    elif (mode == 1):
        if len(contours) != 0:
            c = max(contours, key = cv2.contourArea)
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            if (OldX > 0 and OldY>0):
                blank = cv2.line(blank, (OldX,OldY), (cX,cY), (0,255,0), 2)
            OldX=cX
            OldY=cY
            cv2.imshow('projector',blank)
        else:
            OldX=0
            OldY=0

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
    elif (k==32):
        blank = np.zeros((height,width,3), np.uint8)
        mode=mode+1
        if mode==2:
            mode=0
        
