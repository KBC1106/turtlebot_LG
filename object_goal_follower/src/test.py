#!/usr/bin/env python


from __future__ import print_function
import sys
import rospy
import cv2
import actionlib

#web cam QR
import numpy as np
from pyzbar.pyzbar import decode
import os
import webbrowser


from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import*


from std_msgs.msg import UInt16

PI = 3.1415926535897


class GoalPose:
	x=0.
	y=0.
	theta=0.
	z=0.
	w=0.

class project:
    def __init__(self):
        self.pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1000)
        self.servo_pub=rospy.Publisher('servo', UInt16, queue_size=1000)
        self.servo2_pub=rospy.Publisher('servo2', UInt16, queue_size=1000)
        self.ac=actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #self.ac.wait_for_server() #do 

        
        self.rot=Twist()#cmd_vel control
        self.turn_wise=1

        self.decoding=False# QR code decoding

        self.ball_is_taken=False# ball=object
        self.choose="not" 
        self.cx=0
        self.cy=0
        self.cx_buffer=0

        self.W_buffer1=0
        self.W_buffer2=0
        self.W_buffer3=0

        self.object_w=0
        self.angle=10

        self.bridge = CvBridge()
        self.image_received = False

        self.img_topic = "/camPi/image_raw" #"/camera/rgb/image_raw"
        self.webcam_topic= "/usb_cam/image_raw" #"/camPi/image_raw"   #"/usb_cam"

        #initalize----------------------
        rospy.sleep(1)
        self.grap()
        self.up()
        self.down()
        self.release()
        
        #basic UI-------------------------------
        print("Cargo classify")
        print("start: 1/quit: 0")
        cmd=input(":")			
        if(cmd==1):
            print("start")#1 start
        else:
            print("quit")#2 quit
            self.ac.cancel_goal()				
            exit()

        
        self.test4()
        # self.web_cam()
        # self.test2()
        # self.grap_adv()
        # self.grap()
        # self.up()
        # self.move_to_goal(point)
        # self.down()
        # self.release()

        print("END")

        # Allow up to one second to connection
        rospy.sleep(1)

    def test1(self):
        #object detect and approach
        self.object_apporach()

        #grap and up
        self.grap_adv()
        self.up()

        # back move
        self.linear_mov(-0.4)
    
        #turn
        self.turn(180)

        # down and relase and back
        self.down()
        self.release()
        self.linear_mov(-0.06)

        #turn
        self.turn(120)

        #grap and up
        self.grap_adv()
        self.up()
        #object detect and approach
        self.object_apporach()

        #grap and up
        self.grap_adv()
        self.up()

        # foward move
        self.linear_mov(0.1)

        # down and relase
        self.down()
        self.release()

    def test2(self):
        #grap testing
        self.object_apporach()

        #grap and up
        self.grap_adv()
        #self.up()

    def test3(self):
        #set point
        start=GoalPose()
        green=GoalPose()
        beige=GoalPose()
        [start.x, start.y, start.z, start.w]=[-0.610668178837,-0.193519999971,0.0494708014238,0.998775570289]
        [green.x, green.y, green.z, green.w]=[2.80690463209,-0.851612960387,-0.137289560387,0.990530956916]#green pose
        [beige.x, beige.y, beige.z, beige.w]=[2.61551184282,0.294635420038,-0.0217241247798,0.999764003354]#beige pose
        
        self.turn(180)
        for i in range (0,3):
            #1st object grap
            self.object_apporach()
            self.grap_adv()
            self.up()
            self.linear_mov(-0.4)
            self.turn(180)

            #move to the target point
            if(self.choose=="green"):
                self.move_to_goal(green)
            elif(self.choose=="beige"):
                self.move_to_goal(beige)
            
            #relase and move
            self.down()
            self.release()
            self.linear_mov(-0.1)
            self.turn(180)
            print("remain object: ", 2-i)

            #return to start point
            self.move_to_goal(start)
            self.turn(180)
            rospy.sleep(1)

    def test4(self):
        
        self.object_apporach()
        self.grap_adv()
        self.up()
        self.web_cam()

#usb_cam for QR code detection
    def web_cam(self):
        self.decoding=False
        while(self.decoding==False):
            self.webcam_sub=rospy.wait_for_message(self.webcam_topic, Image, timeout=None)
            self.webcam_callback(self.webcam_sub)
            rospy.sleep(0.1)#0.1sec-> 10hz

    def webcam_callback(self, ros_image):
        print('got an image')
        global bridge
        # convert ros_image into an opencv-compatible image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                ros_image, desired_encoding="passthrough"
            )
        except CvBridgeError as e:
            print(e)

        # OpenCV Code goes after this comment
        # f = open(save_path + "data.txt", "a")
        # t = time.localtime()
        # current_time = time.strftime("%Y-%m-%d %H:%M:%S", t)
        for code in decode(cv_image):
            # print(code.type)
            
            self.decoding=True #controll
            
            data = code.data.decode("utf-8")
            if code.type == "CODE128":
                data = int(data)
                recoveredbytes = data.to_bytes((data.bit_length() + 7) // 8, "little")
                data = recoveredbytes[:-1].decode("utf-8").strip()  # Strip pad after decoding
                
            # else:
            #     data = code.data.decode("utf-8")

            print(data)
            webbrowser.get("firefox").open(data)#data

            # Get geometry of identified barcode/qrcode and draw a rectangle around it
            pts = np.array([code.polygon], np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.polylines(cv_image, [pts], True, (255, 0, 255), 5)

            pts2 = code.rect
            cv2.putText(
                cv_image,
                data,
                (pts2[0], pts2[1]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (255, 0, 255),
                2,
            )
            # f.write(current_time + " ; " + data + "\n")

        # f.close()
        # img = cv2.resize(cv_image, (960, 600))
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)




#obejct searching and apporach-------------------------------------------------        
    def object_apporach(self):
        self.rate=rospy.Rate(1)

        #initalize
        self.ball_is_taken=False
        self.choose="not"
        self.cx=0
        self.cy=0
        self.W_buffer1=0
        self.W_buffer2=0
        self.W_buffer3=0
        self.angle=10       #servo base angle

        self.turn_wise=1    #turn orientation
        angle=140

        t_turn=(2*PI*angle/360)/0.1
        t_end=rospy.Time.now()+rospy.Duration(t_turn)

        while (self.ball_is_taken==False):
            self.image_sub=rospy.wait_for_message(self.img_topic, Image, timeout=None)
            self.callback(self.image_sub)
            
            #turning control
            if(t_end<rospy.Time.now()):
                self.turn_wise=-1*self.turn_wise
                t_end=rospy.Time.now()+rospy.Duration(t_turn)
                self.turn(140*self.turn_wise)
            
            rospy.sleep(0.1)#0.1sec-> 10hz
        self.reset()
        rospy.sleep(1)

#navigation moving-------------------------------------
    def move_to_goal(self, goal_point):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map" #'map'
        goal.target_pose.header.stamp = rospy.Time.now()
	
        goal.target_pose.pose.orientation.x=0
        goal.target_pose.pose.orientation.y=0
        goal.target_pose.pose.orientation.z=goal_point.z
        goal.target_pose.pose.orientation.w=goal_point.w

        goal.target_pose.pose.position.x=goal_point.x
        goal.target_pose.pose.position.y=goal_point.y
        goal.target_pose.pose.position.z=0
        print (goal)
        self.ac.send_goal(goal)
        wait=self.ac.wait_for_result()
        
        if not wait:
            print("error!2")
			
			#rospy.logerr("fail")
			#rospy.signal_shutdown("fail")
        else:
            rospy.sleep(1)

#camera data processing-----------------------------------
    def callback(self, data):

        # Convert image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.image = cv_image
        #self.show_image(cv_image)
        
        self.find_ball(cv_image)
        self.move_to_object()

        
#servo--------------------------------------
    def grap(self):
        print("grap")
        self.servo_pub.publish(170)
        rospy.sleep(1)

    def release(self):
        print("release")
        self.servo_pub.publish(10)
        rospy.sleep(1)
    
    def up(self):
        print("up")
        self.servo2_pub.publish(140)
        rospy.sleep(1)#(sec)
    
    def down(self):
        print("down")
        self.servo2_pub.publish(15)
        rospy.sleep(1)#(sec)

    def grap_adv(self):
        print("grap_adv")
        self.servo_pub.publish(self.angle)
        rospy.sleep(1)

#linear moving---------------------------
    def linear_mov(self, x):
        #initialize
        self.rot.angular.x = self.rot.angular.y = self.rot.angular.z = 0.0
        self.rot.linear.x = self.rot.linear.y  = self.rot.linear.z  = 0.0

        #set speed
        if(x>=0):
            self.rot.linear.x=0.1
        elif(x<0):
            self.rot.linear.x=-0.1

        #compute time
        t_move=abs(x)/0.1
        t_end=rospy.Time.now()+rospy.Duration(t_move)
        
        #executing	
        while(rospy.Time.now()<t_end):
            print("moving")	
            self.pub.publish(self.rot)
            rospy.sleep(0.01)
        
        #stop
        self.rot.linear.x = 0	
        self.pub.publish(self.rot)
        rospy.sleep(1)
    
#turn------------------------------------
    def turn(self, angle):# normal angle
        #initialize
        self.rot.angular.x = self.rot.angular.y = self.rot.angular.z = 0.0
        self.rot.linear.x = self.rot.linear.y  = self.rot.linear.z  = 0.0

        #set direction of turn
        if(angle<0):		
            wise=-1	#cw
        else:
            wise=1	#ccw	
        angle=abs(2*angle*PI/360)

        #set speed
        self.rot.angular.z=2*wise

        #compute time
        t_turn=angle/2	
        t_end=rospy.Time.now()+rospy.Duration(t_turn)
            
        #executing
        print("turn")	
        while(rospy.Time.now()<t_end):
            print("turning")				
            self.pub.publish(self.rot)
            rospy.sleep(0.01)

        #stop
        self.rot.angular.z = 0	
        self.pub.publish(self.rot)
        rospy.sleep(1)
        
    def show_image(self,img):
        #show image
        cv2.imshow("Image Window", img)
        cv2.waitKey(3)
    
    
    def find_goal(self,img):
        hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        hsv_frame = cv2.resize(hsv_frame,(640,300))
        img = cv2.resize(img,(640,300))

        
        
        low_H=25
        low_S=100
        low_V=100
        high_H=32
        high_S=255
        high_V=255


        mask_frame=cv2.inRange(hsv_frame, (low_H, low_S, low_V), (high_H, high_S, high_V))
        cv2.imshow("mask",mask_frame)
        #contours, hierarchy = cv2.findContours(mask_frame,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        mask, contours, hierarchy = cv2.findContours(mask_frame,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        #_, contours, _= cv2.findContours(mask_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        X,Y,W,H=0,0,0,0


        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            
            if(area > 30):
                
                x, y, w, h = cv2.boundingRect(contour)
                if(w*h>W*H):
                    X, Y, W, H= x, y, w, h

        img = cv2.rectangle(img, (X, Y),(X + W, Y + H),(0, 0, 255), 2)

        self.cx = X+(W/2)
        self.cy = Y+(W/2)
        
        print("to goal")
        print(self.cx)
        cv2.imshow("window", img)
        cv2.waitKey(3)
        
        
    def find_ball(self,img):
        hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        hsv_frame = cv2.resize(hsv_frame,(640,300))#300
        hsv_frame = hsv_frame[120:300, 0:640]
        img = cv2.resize(img,(640,300))
        img = img[120:300, 0:640]


        #green
        low_H=30
        low_S=40
        low_V=25
        high_H=100
        high_S=170
        high_V=120

        #black
        # low_H=0
        # low_S=0
        # low_V=0
        # high_H=180
        # high_S=50
        # high_V=50

        #beige2
        low_H2=0
        low_S2=40
        low_V2=50
        high_H2=15
        high_S2=150
        high_V2=150

        #beige
        # low_H=0
        # low_S=40
        # low_V=50
        # high_H=15
        # high_S=150
        # high_V=150

        #white?
        #low_H=0
        #low_S=0
        #low_V=150
        #high_H=180
        #high_S=20
        #high_V=255


        #low_H=0
        #low_S=100
        #low_V=100
        #high_H=18
        #high_S=255
        #high_V=255
        
        #HSV color 
        #low_H=25
        #low_S=100
        #low_V=100
        #high_H=32
        #high_S=255
        #high_V=255

        #mask frame additional
        mask_frame=cv2.inRange(hsv_frame, (low_H, low_S, low_V), (high_H, high_S, high_V))
        mask_frame2=cv2.inRange(hsv_frame, (low_H2, low_S2, low_V2), (high_H2, high_S2, high_V2))

        cv2.imshow("mask2", mask_frame2)
        cv2.imshow("mask",mask_frame)
        # contours, hierarchy = cv2.findContours(mask_frame,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        c, contours, hierarchy = cv2.findContours(mask_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        c2, contours2, hierarchy2 = cv2.findContours(mask_frame2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        x, y, w, h = 0,0,0,0
        X,Y,W,H=0,0,0,0
        area = 0
        area1 = 0
        area2 = 0

        #area estimation one time
        if(self.choose=="not"):
            for pic, contour in enumerate(contours):
                
                area1 = cv2.contourArea(contour)


            for pic, contour2 in enumerate(contours2):
                
                area2= cv2.contourArea(contour2)    

            if(area1>area2):
                self.choose="green"
            else:
                self.choose="beige"

        elif(self.choose=="green"):
            
            for pic, contour in enumerate(contours):
            
                area1 = cv2.contourArea(contour)
            
                if(area1>30):#30
                    
                    x, y, w, h = cv2.boundingRect(contour)
                    if(w*h>W*H):
                        X, Y, W, H= x, y, w, h
            print("green_witdh: ", W)
            print("area: ", area1)      

        elif(self.choose=="beige"):
            
            for pic, contour2 in enumerate(contours2):
                area2= cv2.contourArea(contour2)

                if(area2>30):#30
                    
                    x, y, w, h = cv2.boundingRect(contour2)
                    if(w*h>W*H):
                        X, Y, W, H= x, y, w, h
            print("beige_witdh: ", W)
            print("area: ", area2) 

        img = cv2.rectangle(img, (X, Y),(X + W, Y + H),(0, 0, 255), 2)
        self.cx = X+(W/2)
        self.cy = Y+(W/2)
        print("center: ",self.cx)

        #servo
        if((self.W_buffer1+self.W_buffer2+self.W_buffer3)>1300):
            self.angle=130
        else:
            self.angle=165

        if((self.cx<100)&(self.cx_buffer>100)):
            self.ball_is_taken=True
            print("apporached object!")

        self.cx_buffer=self.cx
        self.W_buffer1=W
        self.W_buffer2=self.W_buffer1
        self.W_buffer3=self.W_buffer2
        
        cv2.imshow("window", img)
        cv2.waitKey(3)

    def move_to_object(self):
        
        if(self.cx==0):
        #non founded
            text="searching"
            self.rot.angular.z=0.1*self.turn_wise
            self.rot.linear.x=0
        #pi/2=90    

        else:  
         #found
            obj_x=self.cx-320

            if(obj_x<=40 and obj_x>=-40):
                text="straight"
                self.rot.angular.z=0
                if(self.W_buffer1<100):
                    self.rot.linear.x=0.3#0.5
                elif(self.W_buffer1>=100 and self.W_buffer1<200):
                    self.rot.linear.x=0.2
                elif(self.W_buffer1>=200):
                    self.rot.linear.x=0.1
    
            elif(obj_x>60):
                text="Left"
                self.rot.angular.z=-0.1
                self.rot.linear.x=0
            elif(obj_x<-60):
                text="Right"
                self.rot.angular.z=0.1
                self.rot.linear.x=0


        self.pub.publish(self.rot)
        #print(text)

    def reset(self):
        self.rot.angular.z=0
        self.rot.linear.x=0
        self.pub.publish(self.rot)


if __name__ == '__main__':

    # Initialize
    rospy.init_node('test', anonymous=False)
    camera = project()
    camera.reset()

