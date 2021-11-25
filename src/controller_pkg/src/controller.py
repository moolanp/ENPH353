#! /usr/bin/python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import imutils

import tensorflow as tf
from tensorflow.keras import models
from tensorflow.python.keras.backend import set_session
from tensorflow.python.keras.models import load_model

#Used to generate images#######
import time
import csv
rows = []
global start_time
#Used for printing multiple images to desktop - only utilized during testing
number = 1
num2 = 0
#######################

sess1 = tf.Session()    
graph1 = tf.get_default_graph()
set_session(sess1)

rospy.init_node('topic_publisher')
rate = rospy.Rate(1)

pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
license_plate_pub = rospy.Publisher('/license_plate', String, queue_size=1)

bridge = CvBridge()

clock_wise = True
height = 720
width = 1280

# Parking Number Spot
Pspot = 2

# Load CNN Model
conv_model = models.load_model('/home/fizzer/ros_ws/src/cnn_trainer/Models/my_model.h5')


###############################################
#Paul Moolan, Steven Brown

team_ID = '20062506,'
password = 'password,'

###############################################


#IMAGE
def image_callback(msg):
	try:
		cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
	except CvBridgeError, e:
		print(e)
	else:
		cv2.imwrite('camera_image.jpeg', cv2_img)

	gray = cv2.cvtColor(cv2_img,cv2.COLOR_BGR2GRAY)

	#Check for Pedestrian/Crosswalk
	if(checkRed(cv2_img)):
		person = checkPerson(cv2_img)
		move_bot(x=0,y=0,z=0)
		if(person):
			move_bot(x=0.5,y=0,z=0)
			rospy.sleep(0.5)
		
	#PID Drive - (Currently Outer Loop Only)		
	else:
		crop_img= gray[(int)(0):height, 640:]
		mask3 = cv2.inRange(crop_img, 250, 255)
		xcm,ycm = centerOfMass(mask3)

		if(xcm == 99999 and ycm == 99999):
			if(clock_wise):
				move_bot(x=0,y=0,z=-0.5)
			else:
				move_bot(x=0,y=0,z=0.5)

		else:
			error = 380-xcm
			global clock_wise
			if(error<0):
				clock_wise = True
			else:
				clock_wise = False

			linear = 0.5-0.0015*error
			turn = 0.0085*error
			move_bot(x=linear,y=0,z=turn)

	#Check for plates in image
	if(np.sum(gray==0) > 5):
		#Find CM of P(1-6) Pixels
		mask = cv2.inRange(gray,0,0)
		# print(np.sum(mask==255))
		x,y = centerOfMass(mask)

		#Masks Blue(Bright and Dark) then blurs to remove noise
		maskblue = cv2.inRange(cv2_img, (100, 0, 0), (255, 100, 100))
		mblue = cv2.medianBlur(maskblue,7)
		xleft = 0
		xright = 0
		ytop = 0
		ybot = 0

		#Calculates boundaries of plate
		for i in range(x,0,-1):
			if(mblue[y][i] == 255):
				xleft = i
				break

		for i in range(x,width):
			if(mblue[y][i] == 255):
				xright = i
				break

		for j in range(y,0,-1):
			if(mblue[j][xright+5] != 255):
				ytop = j
				break

		for j in range(y,height):
			if(mblue[j][xright+5] != 255):
				ybot = j
				break

		#Segments plates into chars and saves to desktop
		if(xleft != 0 and xright != 0 and ytop != 0 and ybot !=0):
			crop_plate= cv2_img[ytop:ybot, xleft:xright]
			segment_chars(crop_plate)
  

##########
#MOVEMENT
##########

def move_bot(x=0, y=0, z=0):
	move = Twist()
	move.linear.x = x
	move.linear.y = y
	move.angular.z = z
	pub.publish(move)

def orient_from_start_into_outer_loop():
	move_bot(x=0.15,y=0,z=0)
	rospy.sleep(2.3)
	turn90(True)
	move_bot()

def turn90(CCW):
	if(CCW):
		move_bot()
		move_bot(x=0,y=0,z=0.85)
		rospy.sleep(2.2)
		move_bot()
	else:
		move_bot(x=0,y=0,z=-0.85)
		rospy.sleep(2.2)
		move_bot()

#######
#IMAGE
#######

#Requires a grayscale thresholded image(ie black and white pixels)
def centerOfMass(grayImg):
	mass_x, mass_y = np.where(grayImg == 255)
	cent_y = np.average(mass_x)
	cent_x = np.average(mass_y)
	
	if(np.sum(grayImg == 255) == 0):
		print("No white pixels in image")
		return (99999,99999)

	return((int)(cent_x),(int)(cent_y))

def checkRed(img_color):
	mask3 = cv2.inRange(img_color, (0,0,255), (0,0,255))
	red_pix = np.sum(mask3 == 255)

	if(red_pix >= 1200):
		return True
	else:
		return False

def checkPerson(img_color):
	# width = 1280, height = 720
	imgCrop = img_color[410:480 , 560:760]
	#cv2.imwrite('/home/fizzer/Desktop/CROPPEDimage1.png', imgCrop)  
	maskPants = cv2.inRange(imgCrop, (50,0,0), (55,50,50))
	blue_pix = np.sum(maskPants == 255)
	move_bot(x=0,y=0,z=0)

	if(blue_pix >= 10):
		return True
	else:
		return False

def convertBack(Y_set):
	L = ['A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R',
	'S','T','U','V','W','X','Y','Z','0','1','2','3','4','5','6','7','8','9']
	Y_new = []
	print(Y_set)
	for vec in Y_set:
		x = np.argmax(vec)
		Y_new.append(L[x])
	return Y_new

def segment_chars(plate):

	#This bit catches any non-plate images that crept in
	height_initial, w, c = plate.shape
	if(height_initial<50):
		return

	#Resizes image, gives better resolution for viewing cropped segments
	thresh =imutils.resize(plate, width=400)
	height_plate, width_plate,channels = thresh.shape
	
	#Manual crop based on ratios of char location
	crop_thresh2 = thresh[0:height_plate,(int)(width_plate/2):width_plate]
	crop_thresh1 = thresh[0:height_plate,0:(int)(width_plate/2)]

	char1 = crop_thresh1[(int)(65*height_plate/100):height_plate,0:(int)(width_plate/4)]
	char2 = crop_thresh1[(int)(65*height_plate/100):height_plate,(int)(width_plate/4):(int)(width_plate/2)]
	char3 = crop_thresh2[(int)(65*height_plate/100):height_plate,0:(int)(width_plate/4)]
	char4 = crop_thresh2[(int)(65*height_plate/100):height_plate,(int)(width_plate/4):(int)(width_plate/2)]

	parkingNumber = thresh[(int)(35*height_plate/100):(int)(65*height_plate/100),(int)(width_plate/2):width_plate]

	char1_resize = cv2.resize(char1, (75,135), interpolation= cv2.INTER_LINEAR)
	char2_resize = cv2.resize(char2, (75,135), interpolation= cv2.INTER_LINEAR)
	char3_resize = cv2.resize(char3, (75,135), interpolation= cv2.INTER_LINEAR)
	char4_resize = cv2.resize(char4, (75,135), interpolation= cv2.INTER_LINEAR)

	# X_dataset_orig = []
	# X_dataset_orig.append(char1_resize)
	# X_dataset_orig.append(char2_resize)
	# X_dataset_orig.append(char3_resize)
	# X_dataset_orig.append(char4_resize)

	# X_dataset = np.array(X_dataset_orig)/255.
	# print(len(X_dataset))

	# global conv_model
	# global sess1
	# global graph1
	# with graph1.as_default():
 #   		set_session(sess1)
	# 	y_pred = conv_model.predict(X_dataset)
	# 	print(len(y_pred))
	# y_pred_as_char = convertBack(y_pred)

	# send_plates(y_pred_as_char)


	##########################
	global number
	global num2
	try:
		if(time.time()-start_time>2):
			number += 1
			if(number == 6):
				number = 0
	except NameError:
		print()

	cv2.imwrite('/home/fizzer/Desktop/P{:01d}'.format(number+1)+str(rows[number])[2:6]+'{:01d}.png'.format(num2), plate) 
	global start_time
	start_time = time.time()

	############################

	# cv2.imwrite('/home/fizzer/Desktop/plate{:03d}char1.png'.format(number), char1_resize)
	# cv2.imwrite('/home/fizzer/Desktop/plate{:03d}char2.png'.format(number), char2_resize)
	# cv2.imwrite('/home/fizzer/Desktop/plate{:03d}char3.png'.format(number), char3_resize)
	# cv2.imwrite('/home/fizzer/Desktop/plate{:03d}char4.png'.format(number), char4_resize) 
	# cv2.imwrite('/home/fizzer/Desktop/plate{:03d}parkingnum.png'.format(number), parkingNumber)

def send_plates(chars):
	plateSTR = ""
	print(chars)
	# global Pspot
	# for i in range(0, len(chars)):
	# 	plateSTR = plateSTR + str(chars[i])
	# 	#license_plate_pub.publish(team_ID + password + str(Pspot) + plateSTR)
	# Pspot += 1
	# if(Pspot == 7):
	# 	Pspot = 1
	print(team_ID + password + str(Pspot) + plateSTR)

		
##########
#MAIN LOOP
##########

#Stop bot and send -1st License Plate
def stop():
	move_bot(0,0,0)
	license_plate_pub.publish(team_ID + password + '-1,' + 'ABCD')
	rospy.sleep(1)

def main():

	#####################################
	global rows
	with open("/home/fizzer/ros_ws/src/2020_competition/enph353/enph353_gazebo/scripts/plates.csv", 'r') as file:
		csvreader = csv.reader(file)
		i=0
		for row in csvreader:
			rows.append(row)
			i+=1
			if(i == 6):
				break
	print(rows)
	#######################################

	#Start - Send 0th License Plate
	rate.sleep()
	license_plate_pub.publish(team_ID + password + '0,' + 'ABCD')

	#Orient into outer loop from start
	rospy.sleep(1)
	orient_from_start_into_outer_loop()

	#DO STUFF AND GET LICENSE PLATES
	rospy.sleep(1)
	rospy.Subscriber("/R1/pi_camera/image_raw", Image, image_callback)
	rospy.spin()
	
	#Make sure to put stop clause at end 


if __name__ == '__main__':
    main()