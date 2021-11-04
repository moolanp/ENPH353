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

bridge = CvBridge()

height = 720
width = 1280


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


	if(checkRed(cv2_img)):
		#Hits Crosswalk-check for pedestrian,
		print("Red")
		move_bot(x=0.2,y=0,z=0)
		

	else:
		gray = cv2.cvtColor(cv2_img,cv2.COLOR_BGR2GRAY)
		crop_img= gray[(int)(0):height, 640:]
		mask3 = cv2.inRange(crop_img, 250, 255)
		xcm,ycm = centerOfMass(mask3)
		error = 380-xcm

		linear = 0.1-0.00035*error
		turn = 0.001*error
		move_bot(x=linear,y=0,z=turn)

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
		return

	return((int)(cent_x),(int)(cent_y))

def checkRed(img_color):
	mask3 = cv2.inRange(img_color, (0,0,255), (0,0,255))
	red_pix = np.sum(mask3 == 255)

	if(red_pix >= 50):
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

