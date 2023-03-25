import socket
from time import sleep
import signal		
import sys
import cv2
import numpy as np
from pyzbar.pyzbar import decode

cap= cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)

myData = ""
temp = "0"
def signal_handler(sig, frame):
    print('Clean-up !')
    cleanup()
    sys.exit(0)

def cleanup():
    s.close()
    print("cleanup done")

ip = "192.168.238.238"     #Enter IP address of laptop after connecting it to WIFI hotspot


#We will be sending a simple counter which counts from 1 to 10 and then closes the socket



#To undeerstand the working of the code, visit https://docs.python.org/3/library/socket.html

print("hello world")
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((ip, 8002))
    s.listen()
    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")
        while True:
            success, img = cap.read()
            for barcode in decode(img):
                print(barcode.data)
                myData = barcode.data.decode('utf-8')
                
                pts = np.array([barcode.polygon], np.int32)
                pts = pts.reshape((-1, 1, 2))
                cv2.polylines(img, pts, True, (0, 0, 255), 5)
            cv2.imshow('Result', img)
            cv2.waitKey(1)
            if (myData != temp):
                print("sending my data")
                temp = myData
                print(myData)
                conn.sendall(str.encode(str(myData)+ str("*")))

           

            













