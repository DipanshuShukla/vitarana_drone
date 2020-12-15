#!/usr/bin/env python


# 			PUBLICATIONS				SUBSCRIPTIONS


from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2, time
import numpy as np
import rospy

from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Vector3



class detector():

	# Initialise everything
	def __init__(self):
		rospy.init_node('barcode_test') #Initialise rosnode 

		# Subscribing to /edrone/camera/image_raw 
		rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic

		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()

		self.cascade = cv2.CascadeClassifier('cascade.xml')

		# to know when to scan
		self.scan = False

		# Publishing /destination_coordinates, /qr_status
		self.coordinates_pub = rospy.Publisher("/destination_coordinates", Vector3, queue_size=1)
		self.qr_status_pub = rospy.Publisher("/qr_status", Bool, queue_size=1)

		time.sleep(2.5)



	# Callback function of camera topic
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
			#cv2.imshow("Image", self.img)
			#cv2.waitKey(0)
		except CvBridgeError as e:
			print(e)
			return


	# To decode qr code and find destination coordinates
	def detect_marker(self):
		
		# detect marker
		gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

		# image, reject levels level weights.
		logo = self.cascade.detectMultiScale(gray, scaleFactor=1.05)

		for (x, y, w, h) in logo:
		    cv2.rectangle(self.img, (x, y), (x + w, y + h), (255, 255, 0), 2)
		cv2.imshow("detected", self.img)
		cv2.waitKey(1)


		


if __name__ == '__main__':
	image_proc_obj = detector()
	r = rospy.Rate(10)
	
	# running appropriate function continuously in loop
	while not rospy.is_shutdown():
		image_proc_obj.detect_marker()
		r.sleep()