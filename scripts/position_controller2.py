#!/usr/bin/env python


from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix, LaserScan
from std_msgs.msg import Float32, Bool, String, Int32, Int32MultiArray
import rospy
import time, math
from copy import deepcopy
from geometry_msgs.msg import Vector3
from vitarana_drone.srv import Gripper



class Control:
	def __init__(self):
		rospy.init_node(
			"position_controller"
		)  # initializing ros node with name drone_control

		self.destination = [0.0, 0.0, 0.0]
        self.destination_cmd = None

        self.mission_status = Bool()
        self.mission_status.data = False

		
		# for bug0 algorithm
		self.distances = [10.0, 10.0, 10.0, 10.0] # [front, right, rear, left]

		# for scanning the qrcode and picking the package
		self.qr_command = Bool()
		self.qr_command.data = False
		self.qr_scanned = False


		self.package_pickable = False
		self.gripper_success = False
		self.package_picked = False
		self.crawling = False

		# wait for the gripper sevice to be running
		rospy.wait_for_service('/edrone/activate_gripper')
		# connection to the gripper service
		self.gripper_service = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)

		
		time.sleep(4)

		# Drone commands
		self.control_cmd = edrone_cmd()
		self.control_cmd.rcRoll = 1500
		self.control_cmd.rcPitch = 1500
		self.control_cmd.rcYaw = 1500
		self.control_cmd.rcThrottle = 1000
		# aux1 to enable/disable drone
		self.control_cmd.aux1 = 2000

		# variables for pid
		# initial setting of Kp, Kd and Ki for throttle
		self.Kp = [720000, 720000, 396.0]
		self.Ki = [0.66, 0.66, 0.88]
		self.Kd = [20000000, 20000000, 4000.0]

		self.prev_value = [0.0, 0.0, 0.0]  # [lat, long, alt]
		self.max_value = 2000
		self.min_value = 1000

		self.error = [0.0, 0.0, 0.0]  # [lat, long, alt]
		self.p_error = [0.0, 0.0, 0.0]  # [lat, long, alt]
		self.dif_error = [0.0, 0.0, 0.0]
		self.sum_error = [0.0, 0.0, 0.0]  # Iterm

		self.output = [0.0, 0.0, 0.0]  # [lat, long, alt]

		self.p_error_limit = 0.000036
		self.p_error_limit_altitude = 2

		# Publishing /edrone/drone_command, /altitude_error, /zero_error, /qr_command
		self.cmd_pub = rospy.Publisher("/drone_command", edrone_cmd, queue_size=1)
		self.altitude_pub = rospy.Publisher("/altitude_error", Float32, queue_size=1)
		self.zero_pub = rospy.Publisher("/zero_error", Float32, queue_size=1)
		self.qr_command_pub = rospy.Publisher("/qr_command", Bool, queue_size=1)

		# Subscribing to /edrone/gps, /edrone/gripper_check, /destination_coordinates, /qr_status, /edrone/range_finder_top
		rospy.Subscriber("/edrone/gps", NavSatFix, self.gps_callback)
		rospy.Subscriber("/destination_coordinates", Vector3, self.destination_coordinates_callback)
		rospy.Subscriber("/edrone/gripper_check", String, self.gripper_check_callback)
		rospy.Subscriber("/qr_status", Bool, self.qr_status_callback)
		#rospy.Subscriber("/pid_tuning_altitude", PidTune, self.longitude_set_pid)
		rospy.Subscriber("/edrone/range_finder_top", LaserScan, self.range_finder_callback)

		# to turn on the drone
		self.cmd_pub.publish(self.control_cmd)

		# to delay time in the location for accuracy
		self.arival_time = 0

	# destination coordinates callback function
	def destination_coordinates_callback(self, msg):
		if self.qr_scanned:
			self.drop_location = [msg.x, msg.y, msg.z]
			
			# adding scanned location to the location setpoint list
			if not self.drop_location == self.location_setpoints[-2]:
				self.location_setpoints.append([self.box_location[0], self.box_location[1], self.box_location[2] + 2]) # hover at a height
				#self.location_setpoints.append([self.drop_location[0], self.location_setpoints[-1][1], self.location_setpoints[-1][2]])
				self.location_setpoints.append([self.drop_location[0], self.drop_location[1], self.location_setpoints[-1][2]])
				self.location_setpoints.append([self.drop_location[0], self.drop_location[1], self.drop_location[2] + 2])
				self.location_setpoints.append(self.drop_location)
				self.location_setpoints.append([self.drop_location[0], self.drop_location[1], self.drop_location[2] + 2])



	# qr status callback function
	def qr_status_callback(self, msg):
		self.qr_scanned = msg.data

	# GPS callback function
	def gps_callback(self, msg):
		self.drone_position[0] = msg.latitude
		self.drone_position[1] = msg.longitude
		self.drone_position[2] = msg.altitude

		# print(self.drone_position)

	
	# range finder callback function
	def range_finder_callback(self, msg):
		for i in range(4):
			self.distances[i] = msg.ranges[i]

	def gripper_check_callback(self, msg):
		self.package_pickable = msg.data
	
	def latitude_set_pid(self, latitude):
		self.Kp[0] = latitude.Kp * 500
		self.Ki[0] = latitude.Ki * 0.001
		self.Kd[0] = latitude.Kd * 8000

	def longitude_set_pid(self, longitude):
		#self.Kp[1] = longitude.Kp * 500
		self.Ki[1] = longitude.Ki * 0.001
		#self.Kd[1] = longitude.Kd * 8000

	def altitude_set_pid(self, altitude):
		self.Kp[2] = altitude.Kp * 0.4
		self.Ki[2] = altitude.Ki * 0.002
		self.Kd[2] = altitude.Kd * 0.8

	# function to pick package up
	def pick_package(self):
		if self.package_pickable:
			while not self.gripper_success:
				self.gripper_success = self.gripper_service(True)
				if self.gripper_success:
					self.package_picked = True

	# function to pick package up
	def drop_package(self):
		if self.package_picked:
			self.gripper_success = self.gripper_service(False)
			self.package_picked = False

	def set_waypiont(self):
				# to move to next location
		# checking tolerance in lat, long, alt respectively
		if (
			(self.error[0] > -0.000004517 and self.error[0] < 0.000004517)
			and (self.error[1] > -0.0000047487 and self.error[1] < 0.0000047487)
			and (self.error[2] > -0.2 and self.error[2] < 0.2)
		):

			if not self.arival_time:
				self.arival_time = rospy.Time.now().to_sec()

			elif rospy.Time.now().to_sec() - self.arival_time > 0.5 and (
				(self.error[0] > -0.000004517 and self.error[0] < 0.000004517)
				and (self.error[1] > -0.0000047487 and self.error[1] < 0.0000047487)
				and (self.error[2] > -0.2 and self.error[2] < 0.2)
			):

				self.arival_time = 0

				if self.location_setpoints[self.location_index] == self.box_location:
					# to command the gripper to pick package
					self.pick_package()
					self.location_index += 1

				elif self.location_setpoints[self.location_index] == self.drop_location:
					# to command the gripper to drop package
					self.drop_package()
					self.location_index += 1

				elif self.location_index == len(self.location_setpoints) - 2:
					
					if self.qr_scanned:
						self.location_index += 1
						self.qr_command.data = False
					else:
						self.qr_command.data = True


				elif not self.location_index == len(self.location_setpoints) - 1:

					#print("Location {} reached.".format(self.location_index + 1))

					self.location_index += 1

				elif self.control_cmd.aux1 == 2000 and False:

					self.control_cmd.aux1 = 1000

					print("Final location reached.")

					#print("Tolerance:")
					#print("lat = {}".format(self.error[0]))
					#print("long = {}".format(self.error[1]))
					#print("alt = {}".format(self.error[2]))


	# to check for an obstacle
	def obstacle_encountered(self):
		if self.distances[3] < 6:
			return True
		return False

	def metres_to_lat(self, lat):
		return (lat / 110692.0702932625)

	def meters_to_long(self, lon):
		return (lon / -105292.0089353767)

	def bug0_crawl(self):
		if self.obstacle_encountered():
			print("crawling like a bug")
#			if self.distances[3] < 4.4:
#				self.error[0] = (4.4 - self.distances[3]) * 0.000004
#			else:
#				self.error[0] = 0.0
#			self.error[1] = 0.00004

			#if self.distances[3] < 4.5:
			self.error[0] = self.metres_to_lat(4.6 - self.distances[3])
			self.error[1] = - self.meters_to_long(1.8)

	def cmd(self):

		# Computing error (for proportional)
		for i in range(3):


			self.error[i] = (
				self.location_setpoints[self.location_index][i] - self.drone_position[i]
			)
			if self.obstacle_encountered():
				self.bug0_crawl()

			if i != 2:
				if self.error[i] > self.p_error_limit:
					self.p_error[i] = self.p_error_limit
				elif self.error[i] < -self.p_error_limit:
					self.p_error[i] = -self.p_error_limit
				else:
					self.p_error[i] = self.error[i]
			else:
				if self.error[i] > self.p_error_limit_altitude:
					self.p_error[i] = self.p_error_limit_altitude
				elif self.error[i] < -self.p_error_limit_altitude:
					self.p_error[i] = -self.p_error_limit_altitude
				else:
					self.p_error[i] = self.error[i]


			# change in error (for derivative)
			self.dif_error[i] = self.error[i] - self.prev_value[i]

			# sum of errors (for integral)
			self.sum_error[i] = (self.sum_error[i] + self.error[i]) * self.Ki[i]

			# calculating the pid output required for throttle
			self.output[i] = (
				(self.Kp[i] * self.p_error[i])
				+ self.sum_error[i]
				+ (self.Kd[i] * self.dif_error[i])
			)

		# Setting control values
		self.control_cmd.rcRoll = self.output[0] + 1500
		self.control_cmd.rcPitch = self.output[1] + 1500
		self.control_cmd.rcThrottle = self.output[2] + 1500

		# print(self.output)

		# Limiting the output value and the final command value
		if self.control_cmd.rcRoll > self.max_value:
			self.control_cmd.rcRoll = self.max_value

		elif self.control_cmd.rcRoll < self.min_value:
			self.control_cmd.rcRoll = self.min_value

		if self.control_cmd.rcPitch > self.max_value:
			self.control_cmd.rcPitch = self.max_value

		elif self.control_cmd.rcPitch < self.min_value:
			self.control_cmd.rcPitch = self.min_value

		if self.control_cmd.rcYaw > self.max_value:
			self.control_cmd.rcYaw = self.max_value

		elif self.control_cmd.rcYaw < self.min_value:
			self.control_cmd.rcYaw = self.min_value

		if self.control_cmd.rcThrottle > self.max_value:
			self.control_cmd.rcThrottle = self.max_value

		elif self.control_cmd.rcThrottle < self.min_value:
			self.control_cmd.rcThrottle = self.min_value

		# Update previous error value
		for i in range(3):
			self.prev_value[i] = self.error[i]

		self.cmd_pub.publish(self.control_cmd)
		self.altitude_pub.publish(self.error[0])

		self.zero_pub.publish(0.0)

		self.qr_command_pub.publish(self.qr_command)

		# to keep track of and update destination
		self.set_waypiont()



if __name__ == "__main__":
	Controller = Control()
	r = rospy.Rate(20)
	while not rospy.is_shutdown():
		Controller.cmd()
		r.sleep()
