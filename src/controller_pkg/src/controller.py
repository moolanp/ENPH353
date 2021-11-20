#! /usr/bin/python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np

rospy.init_node('topic_publisher')
rate = rospy.Rate(1)

pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=1)
license_plate_pub = rospy.Publisher('/license_plate', String, queue_size=1)

clock_wise = True

bridge = CvBridge()

height = 720
width = 1280

#Used for printing multiple images to desktop
number = 0


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
		# print(person)
		move_bot(x=0,y=0,z=0)
		if(person):
			move_bot(x=0.4,y=0,z=0)
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

			linear = 0.4-0.001*error
			turn = 0.007*error
			move_bot(x=linear,y=0,z=turn)

	#Check for plates in image
	if(np.sum(gray==0) > 10):
		#Find CM of P(1-6) Pixels
		mask = cv2.inRange(gray,0,0)
		print(np.sum(mask==255))
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

		#Draws a circle on each boundary to test and prints all plates to desktop to be checked
		if(xleft != 0 and xright != 0 and ytop != 0 and ybot !=0):
			cv2.circle(cv2_img,(xleft,y), 10, (0,0,255),-1)
			cv2.circle(cv2_img,(xright,y), 10, (0,0,255),-1)
			cv2.circle(cv2_img,(x,y), 10, (0,0,255),-1)
			cv2.circle(cv2_img,(x,ytop), 10, (0,0,255),-1)
			cv2.circle(cv2_img,(x,ybot), 10, (0,0,255),-1)

			cv2.imwrite('/home/fizzer/Desktop/CROPPEDimage{:03d}.png'.format(number), cv2_img)
			global number
			number += 1   


#MOVEMENT

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

		
#MAIN

#Stop bot and send -1st License Plate
def stop():
	move_bot(0,0,0)
	license_plate_pub.publish(team_ID + password + '-1,' + 'ABCD')
	rospy.sleep(1)

def main():

	#Start and send 0th License Plate
	rate.sleep()
	license_plate_pub.publish(team_ID + password + '0,' + 'ABCD')

	#Orient into outer loop from start
	rospy.sleep(1)
	orient_from_start_into_outer_loop()

	#DO STUFF AND GET LICENSE PLATES
	rospy.sleep(2)
	rospy.Subscriber("/R1/pi_camera/image_raw", Image, image_callback)
	rospy.spin()
	
	#Make sure to put stop clause at end 


if __name__ == '__main__':
    main()