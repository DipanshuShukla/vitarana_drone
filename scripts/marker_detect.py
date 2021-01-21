#!/usr/bin/env python


# 			PUBLICATIONS				SUBSCRIPTIONS
#			/edrone/err_x_m 			/edrone/camera/image_raw
#			/edrone/err_y_m 			/edrone/Z_m_error
#			/edrone/marker_visibility	/edrone/marker_scan_cmd

from vitarana_drone.msg import MarkerData
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2, time
import numpy as np
import rospy, math

from std_msgs.msg import Bool, Float32, Int32, Float32MultiArray
from geometry_msgs.msg import Vector3



class detector():

	# Initialise everything
	def __init__(self):
		rospy.init_node('marker_detector') #Initialise rosnode 

		# Subscribing to /edrone/camera/image_raw 
		rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		rospy.Subscriber("/edrone/Z_m_error", Float32, self.Z_m_callback)
		rospy.Subscriber("/edrone/marker_scan_cmd", Bool, self.marker_scan_cmd_callback)
		rospy.Subscriber("/edrone/curr_marker_id", Int32, self.marker_id_callback)



		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()

		self.cascade = cv2.CascadeClassifier('/home/codebunny/catkin_ws/src/vitarana_drone/scripts/cascade.xml')

		# to know when to scan
		self.scan = False

		self.marker = Float32MultiArray()
		for i in range(4):
			self.marker.data.append(0.0)

		self.detecting_marker = False
		self.err_x_m = Float32()
		self.err_y_m = Float32()
		self.err_x_m.data = None
		self.err_y_m.data = None
		self.img_width = 400
		self.hfov_rad = 1.3962634
		self.focal_length = (self.img_width / 2) / (math.tan(self.hfov_rad / 2))

		self.centre_x_pixel = 0.0
		self.centre_y_pixel = 0.0
		self.Z_m = 0.0

		self.visibility = Bool()
		self.visibility.data = False

		self.scaleFactor = 1.05		


		# Publishing /destination_coordinates, /qr_status
		self.err_x_m_pub = rospy.Publisher("/edrone/err_x_m", Float32, queue_size=1)
		self.err_y_m_pub = rospy.Publisher("/edrone/err_y_m", Float32, queue_size=1)
		self.visibility_pub = rospy.Publisher("/edrone/marker_visibility", Bool, queue_size=1)
		self.marker_data_pub = rospy.Publisher("/edrone/marker_data", MarkerData, queue_size=1)

		# to be published for task 3
		self.marker_data = MarkerData()
		self.marker_data.marker_id = 0
		self.marker_data.err_x_m = 0.0
		self.marker_data.err_y_m = 0.0

		time.sleep(3)


		self.view_img = True
	

	def marker_scan_cmd_callback(self,msg):
		self.scan = msg.data

	def marker_id_callback(self, msg):
		self.marker_data.marker_id = msg.data

	def Z_m_callback(self, msg):
		self.Z_m = msg.data

	# Callback function of camera topic
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
			#cv2.imshow("Image", self.img)
			#cv2.waitKey(0)
		except CvBridgeError as e:
			print(e)
			return
		#print(self.img.shape)


	# To decode qr code and find destination coordinates
	def detect_marker(self):

		# detect marker
		gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)

		# image, reject levels level weights.
		logo = self.cascade.detectMultiScale(gray, scaleFactor=self.scaleFactor)
		#print(logo)
		for (x, y, w, h) in logo:

			if self.view_img:
				cv2.rectangle(self.img, (x, y), (x + w, y + h), (0,255,0), (h+w)/30)
				cv2.circle(self.img,(x + w / 2, y + h / 2), int(round(math.sqrt(w**2 + h**2)) / 2), (0,255,0), (h+w)/32)
				cv2.line(self.img,(x+w,y),(x,y+h),(0,255,0),(h+w)/32)
				cv2.line(self.img,(x,y),(x+w,y+h),(0,255,0),(h+w)/32)

			self.centre_x_pixel = x + w / 2  - 200
			self.centre_y_pixel = y + h / 2 - 200

			
			self.err_x_m.data = (self.centre_x_pixel) * self.Z_m / self.focal_length
			self.err_y_m.data = (self.centre_y_pixel) * self.Z_m / self.focal_length
			#self.err_y_m.data -= 0.35

			self.visibility.data = True

			break
		else:
			#for i in range(4):
			#	self.marker_coordinates.data[i] = 0.0
			self.visibility.data = False

		if self.view_img:

			cv2.line(self.img,(400/2-8,400/2),(400/2+8,400/2),(0,0,255),1)
			cv2.line(self.img,(400/2,400/2-8),(400/2,400/2+8),(0,0,255),1)
			
			cv2.line(self.img,(400/5,400/5),(400/5+15,400/5),(0,0,255),1)
			cv2.line(self.img,(400/5,400/5),(400/5,400/5+15),(0,0,255),1)

			cv2.line(self.img,(400*4/5,400/5),(400*4/5,400/5+15),(0,0,255),1)
			cv2.line(self.img,(400*4/5,400/5),(400*4/5-15,400/5),(0,0,255),1)

			cv2.line(self.img,(400*4/5,400*4/5),(400*4/5,400*4/5-15),(0,0,255),1)
			cv2.line(self.img,(400*4/5,400*4/5),(400*4/5-15,400*4/5),(0,0,255),1)

			cv2.line(self.img,(400/5,400*4/5),(400/5,400*4/5-15),(0,0,255),1)
			cv2.line(self.img,(400/5,400*4/5),(400/5+15,400*4/5),(0,0,255),1)

			cv2.putText(self.img, "Visibility = {}".format(self.visibility.data), (400*5/200,400*190/200), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0,255,0) if self.visibility.data else (0,0,255), 1)
			cv2.putText(self.img, "Scan = {}".format(self.scan), (400*5/200,400*196/200), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0,255,0) if self.scan else (0,0,255), 1)

			cv2.imshow("detected", self.img)
			cv2.waitKey(1)

		#print([self.err_x_m, self.err_y_m])
		#print([self.centre_x_pixel, self.centre_y_pixel])
		
		self.marker_data.err_x_m = self.err_x_m.data
		self.marker_data.err_y_m = self.err_y_m.data

		# publishing data
		if self.visibility.data:		
			self.err_x_m_pub.publish(self.err_x_m)
			self.err_y_m_pub.publish(self.err_y_m)
			self.marker_data_pub.publish(self.marker_data)
		self.visibility_pub.publish(self.visibility)

		


if __name__ == '__main__':
	image_proc_obj = detector()
	r = rospy.Rate(1) # 1hz
	
	# running appropriate function continuously in loop
	while not rospy.is_shutdown():
		image_proc_obj.detect_marker()
		r.sleep()