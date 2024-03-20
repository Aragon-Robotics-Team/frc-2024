# an object of WSGI application  
from __future__ import print_function
from webcam import WebcamVideoStream
from fps import FPS
import argparse
import imutils
import cv2
from flask import Flask, Response, render_template
import apriltag 
import math
import threading 
import sys
import time
from networktables import NetworkTables

# To see messages from networktables, you must setup logging
import logging
		
# ap = argparse.ArgumentParser()
# ap.add_argument("-n", "--num-frames", type=int, default=100,
# help="# of frames to loop over for FPS test")
# ap.add_argument("-d", "--display", type=int, default=-1,
# help="Whether or not frames should be displayed")
# args = vars(ap.parse_args())

# fps = FPS().start() 

vid = cv2.VideoCapture(0)
options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)
NetworkTables.initialize(server='10.8.40.2')
logging.basicConfig(level=logging.DEBUG)	
while True:
	success, frame = vid.read()
	if not success:
		break
	else:
		gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
		results = detector.detect(gray)
		# print("[INFO] {} total AprilTags detected".format(len(results)))


		# loop over the AprilTag detection results
		for r in results:
			# extract the bounding box (x, y)-coordinates for the AprilTag
			# and convert each of the (x, y)-coordinate pairs to integers
			(ptA, ptB, ptC, ptD) = r.corners
			ptB = (int(ptB[0]), int(ptB[1]))
			ptC = (int(ptC[0]), int(ptC[1]))
			ptD = (int(ptD[0]), int(ptD[1]))
			ptA = (int(ptA[0]), int(ptA[1]))
			# draw the bounding box of the AprilTag detection
			# cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
			# cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
			# cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
			# cv2.line(frame, ptD, ptA, (0, 255, 0), 2)
			# draw the center (x, y)-coordinates of the AprilTag
			(cX, cY) = (int(r.center[0]), int(r.center[1]))
			cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
			# draw the tag family on the image
			tagFamily = r.tag_family.decode("utf-8")
			# cv2.putText(frame, tagFamily, (ptA[0], ptA[1] - 15),
			# cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

			# print("[INFO] tag family: {}".format(tagFamily))
			
			#distance from tag
			#fov = math.atan(6.5/12)*(1/2) 
			#fov = 110 #degrees
			fov = 2.05949 #radians
			#FOV = 2 arctan x/2f
			# apriltag_width=6.5 #inches
			# focal_length= 2 * math.tan(fov/2)/(0.25)
			# # focal_length=0.11
			# per_width = int(ptB[0])- int(ptA[0])
			# #per_width= math.sqrt((int(ptB[1])- int(ptA[1]))**2 + (int(ptB[0])- int(ptA[0]))**2)
			
			# distance = apriltag_width * focal_length/per_width
			# print(distance)
			# distance = str(distance)
			# cv2.putText(frame, "Distance: "+ distance, (100,100),
			# cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
			
			#angle = math.atan(cX-(624/2)/focal_length)*(1/2)
			#angle/fov = cX/640
			angle=fov*cX/640
			angle=angle-(1/2)*fov
			angle = str(angle)
			cv2.putText(frame, "Angle:" + angle, (200,200),
			cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
			sd = NetworkTables.getTable("SmartDashboard")
			sd.putNumber("angle", angle)
			a_value = sd.getNumber('robotTime',0)
			print("Angle: {}".format(angle))
			print("Tag_ID: {}".format(r.tag_id)) 

			#v_fov = 89.331
			v_fov = 1.559 #in radians
			v_angle = v_fov*cY/400
			v_angle = v_angle-(1/2)*v_fov
			# opp=2 #random 
			# hyp=2 #random 
			# camera_angle =2*math.atan(opp/hyp)
			distance = 48.03/math.tan(v_angle)
			if r.tag_id==3 and r.tag_id==4 and r.tag_id==7 and r.tag_id==8 and r.tag_id==1 and r.tag_id==2 and r.tag_id==9 and r.tag_id==10:
			 	distance = 48.03/math.tan(v_angle)
			elif r.tag_id==5 and r.tag_id==6:
				distance = 59.97/math.tan(v_angle)
			elif r.tag_id==11 and r.tag_id==12 and r.tag_id==13 and r.tag_id==14 and r.tag_id==15 and r.tag_id==16:
				distance = 47.63/math.tan(v_angle)

			print("Distance: {}".format(distance))
		# show the output image after AprilTag detection
		# ret, buffer = cv2.imencode('.jpg', frame)
		# frame = buffer.tobytes()
		# yield (b'--frame\r\n'
		# 	b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
			# concat frame one by one and show result
		# yield("")

# A decorator used to tell the application 
# which URL is associated function 
# t1 = threading.Thread(gen, arges=(10,))
# t2 = threading.Thread(frames, arges=(10,))

# t1.start()
# t2.start()

# t1.join()
# t2.join()