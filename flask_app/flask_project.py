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

app = Flask(__name__)   # Flask constructor

def frames():
        ap = argparse.ArgumentParser()
        ap.add_argument("-n", "--num-frames", type=int, default=100,
        help="# of frames to loop over for FPS test")
        ap.add_argument("-d", "--display", type=int, default=-1,
        help="Whether or not frames should be displayed")
        args = vars(ap.parse_args())

        fps = FPS().start() 

        vs = WebcamVideoStream(src=0).start()
        while fps._numFrames < args["num_frames"]:
                # grab the frame from the stream and resize it to have a maximum
                # width of 400 pixels
                frame = imutils.resize(frame, width=400)
                fps.update()
                fps.stop()
                print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
                print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

                # loop over some frames...this time using the threaded stream
                print("[INFO] sampling THREADED frames from webcam...")
        while fps._numFrames < args["num_frames"]:
                # grab the frame from the threaded video stream and resize it
                # to have a maximum width of 400 pixels
                frame = vs.read()
                frame = imutils.resize(frame, width=400)

                fps.update()
                # stop the timer and display FPS information
                fps.stop()
                print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
                print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))


def gen():
        vid = cv2.VideoCapture(0)

        options = apriltag.DetectorOptions(families="tag36h11")
        detector = apriltag.Detector(options)
        NetworkTables.initialize(server='10.8.40.2')
        logging.basicConfig(level=logging.DEBUG)        
        while True:
                success, frame = vid.read()
                # print(frame.shape)
                if not success:
                        break
                else:
                        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                        results = detector.detect(gray)
                        # print("[INFO] {} total AprilTags detected".format(len(results)))

                        # loop over the AprilTag detection results
                        if len(results) == 0:
                                sd = NetworkTables.getTable("SmartDashboard")
                                sd.putNumber("TagDetected", 0)
                                a_value = sd.getNumber('robotTime',0)
                                print("False")     
                        else:
                                sd = NetworkTables.getTable("SmartDashboard")
                                sd.putNumber("TagDetected", 1)
                                a_value = sd.getNumber('robotTime',0)
                                print("True")
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
                                #fov = 118 
                                # fov = 2.05949 #radians
                                fov = 1.57
                                # apriltag_width=6.5 #inches
                                # focal_length= (0.25)/(2*math.tan(fov/2))
                                # per_width= math.sqrt((int(ptB[1])- int(ptA[1]))**2 + (int(ptB[0])- int(ptA[0]))**2)
                                
                                # distance = apriltag_width * focal_length/per_width
                                # distance = str(distance)
                                # cv2.putText(frame, "Distance: "+ distance, (100,100),
                                # cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
								 #angle from center
                                #angle = math.atan(cX-(624/2)/focal_length)*(1/2)

                                angle=fov*cX/640
                                angle=angle-(1/2)*fov
                        
                                sd = NetworkTables.getTable("SmartDashboard")
                                sd.putNumber("angle", angle)
                                a_value = sd.getNumber('robotTime',0)
                                print("Angle: {}".format(angle))
                                print("Tag_ID: {}".format(r.tag_id))                            
                                #v_fov = 0.785 
                                v_fov = 0.71
                                #v_fov = 1.256
                                v_angle = v_fov*cY/400
                                v_angle =(-1)*(v_angle - (1/2)*v_fov)
                                camera_angle =0.628
                                camera_height = 7.5
                                #testing_height = 45
                                distance = 0
                                if r.tag_id==4 or r.tag_id==7:
                                        distance = (51.875-camera_height)/math.tan(v_angle+camera_angle)
                                        #distance = (testing_height-camera_height)/math.tan(v_angle+camera_angle)
                                        #distance = (testing_height-camera_height)/(math.tan(v_angle+camera_angle)*math.cos(angle))
                                elif r.tag_id==5 or r.tag_id==6 or r.tag_id==1 or r.tag_id==2 or r.tag_id==9 or r.tag_id==10:
                                        distance = 48.03-camera_height/math.tan(v_angle+camera_angle)
                                elif r.tag_id==11 or r.tag_id==12 or r.tag_id==13 or r.tag_id==14 or r.tag_id==15 or r.tag_id==16:
                                        distance = 47.63-camera_height/math.tan(v_angle+camera_angle)

                                # next steps: camera distortion-- check if angle is accurate
                                print("Distance: {}".format(distance))
                                sdD = NetworkTables.getTable("SmartDashboard")
                                sdD.putNumber("Distance", distance)
                                b_value = sdD.getNumber('robotTime',0)


                                distance = str(distance)
                                cv2.putText(frame, "Distance: "+ distance, (300,300),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                                angle = str(angle)
                                v_angle = v_angle * (180/math.pi) #back to degrees
                                cv2.putText(frame, "Angle:" + angle, (200,200),
                                                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                                v_angle = str(v_angle)
                                cv2.putText(frame, "Vertical Angle: "+ v_angle, (100,100),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                                
                        # show the output image after AprilTag detection
                        ret, buffer = cv2.imencode('.jpg', frame)
                        frame = buffer.tobytes()
                        yield (b'--frame\r\n'
                                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  # concat frame one by one and show result
#A decorator used to tell the application 
# which URL is associated function 
@app.route('/')       
def index():
        return render_template('index.html')

@app.route('/video_feed')
def video_feed():
        return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame') 

if __name__=='__main__': 
        app.run(debug = True,host='0.0.0.0')
        t1 = threading.Thread(gen, arges=(10,))
        t2 = threading.Thread(frames, arges=(10,))
   
        t1.start()
        t2.start()

        t1.join()
        t2.join()
