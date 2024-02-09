import threading
import serial
import time
import cv2
import numpy as np 

url = 'http://192.168.1.107:8080/video'

cap = cv2.VideoCapture(url)
arduino = serial.Serial('COM10', 9600, timeout=1)

x=0
y=0

def send_s():
    arduino.write(b's')
    time.sleep(3)

while True:

    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    lower_green = np.array([40, 50, 50])
    upper_green = np.array([80, 255, 255])

    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    maskeli = cv2.bitwise_and(frame,frame,mask=green_mask)
    contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours: 		
        area = cv2.contourArea(contour)
        if(area > 300):
            x,y,w,h = cv2.boundingRect(contour)
            frame = cv2.rectangle(frame, (x,y),(x+w,y+h),(0,255,0),10)
            durum=""
            if(x>810 and x<1110 and y <330 and y > 0):
                durum="front"
                arduino.write(b'b')
                timer = threading.Timer(0.25, send_s)
                timer.start()
            elif(x>810 and x<1110 and y >750 and y< 1080):
                durum="back"
                arduino.write(b'f')
                timer = threading.Timer(0.25, send_s)
                timer.start()
            elif(x>0 and x<810 and y<1080):
                durum="left"
                arduino.write(b'r')
                timer = threading.Timer(0.25, send_s)
                timer.start()
            elif(x>1110 and x<1920 and y<1080):
                durum="right"
                arduino.write(b'l')
                timer = threading.Timer(0.25, send_s)
                timer.start()
            elif(x>810 and x<1110 and y>330 and y<750):
                durum="stop"
                timer = threading.Timer(0.25, send_s)
                timer.start()
            cv2.putText(frame, durum + ' ' + 'x:' + str(x) + ' ' + 'y:' + str(y)  , (10,50), cv2.FONT_ITALIC,2, 100,5,5) 


    cv2.imshow("filtreli", maskeli)
    cv2.imshow("ana ekran", frame)
    key = cv2.waitKey(1)
    if key == 27:
        break
cv2.destroyAllWindows()
cap.release()