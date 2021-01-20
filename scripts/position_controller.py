#!/usr/bin/env python

#           PUBLICATIONS                SUBSCRIPTIONS
#       /edrone/drone_command           /edrone/gps
#       /altitude_error                 /pid_tuning_altitude
#       /zero_error                     /edrone/range_finder_top
#       /latitude_error                 /edrone/gripper_check
#       /longitude_error                /destination_coordinates
#       /qr_command                     /qr_status
#                                       


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

		self.drone_position = [0.0, 0.0, 0.0]  # [lat, long, alt]

		#self.location_setpoints = [[19.0009248718, 71.9998318945, 22.16 + 4]]  # [lat, long, alt]
		#self.location_setpoints = [[19.0000271, 72, 2.5]] # for box.launch

		#self.box_location = [19.0007046575, 71.9998955286, 22.1599967919]
		#self.box_location = [19.0, 72.0, 0.31] # for box.launch

		#self.location_setpoints = []
		#self.building_coordinates = [[18.9990965928, 72.0000664814, 10.75], [18.9990965925, 71.9999050292, 22.2], [18.9993675932, 72.0000569892, 10.7]]
		#self.building_coordinates = [[18.9993675932, 72.0000569892, 10.7]]


		#self.drop_location = []

		self.building_id = Int32
		self.building_id.data = 1

		self.destination = [0.0, 0.0, 0.0]
		self.destination_cmd = None

		self.mission_status = Bool()
		self.mission_status.data = False


		#self.location_setpoints.append([self.location_setpoints[-1][0], self.box_location[1], self.location_setpoints[-1][2]])

		#self.location_setpoints.append([self.box_location[0], self.box_location[1], self.box_location[2] + 4])
		#self.location_setpoints.append([self.box_location[0], self.box_location[1], self.box_location[2] + 0.6])
		#self.location_setpoints.append(self.box_location)

		#print(self.location_setpoints
		
		# for bug0 algorithm
		self.distances = [10.0, 10.0, 10.0, 10.0, 10.0] # [front, right, rear, left, bottom]
		self.safe_pos = None
		self.safe_pos2 = None
		self.detection_distance = 12
		self.safe_distance = 4
		self.encounter_time = 0.0

		

		# for scanning the qrcode and picking the package
		self.qr_command = Bool()
		self.qr_command.data = False
		self.qr_scanned = False


		self.package_pickable = False
		self.gripper_success = False
		self.package_picked = False
		self.crawling = False

		time.sleep(2)

		# wait for the gripper sevice to be running
		#rospy.wait_for_service('/edrone/activate_gripper')
		# connection to the gripper service
		self.gripper_service = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)

		
		

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

		self.error_check = [0.0, 0.0, 0.0]

		self.output = [0.0, 0.0, 0.0]  # [lat, long, alt]

		self.p_error_limit = [10/110692.0702932625, 2/105292.0089353767, 1.4]


		# for marker detection
		self.time = None
		self.marker_visibility = False
		self.marker_scan = Bool()
		self.marker_scan.data =False
		self.Z_m = Float32()
		self.Z_m.data = 0.0
		self.err_x_m = 0.0
		self.err_y_m = 0.0
		self.err_x = 0.0
		self.err_y = 0.0
		self.search_circle = None
		self.search_index = 0

		self.timeout = None

		self.update_destination = False



		# Publishing /edrone/drone_command, /altitude_error, /zero_error, /qr_command
		self.cmd_pub = rospy.Publisher("/drone_command", edrone_cmd, queue_size=1)
		self.altitude_pub = rospy.Publisher("/altitude_error", Float32, queue_size=1)
		self.zero_pub = rospy.Publisher("/zero_error", Float32, queue_size=1)
		self.qr_command_pub = rospy.Publisher("/qr_command", Bool, queue_size=1)
		self.Z_m_error_pub = rospy.Publisher("/edrone/Z_m_error", Float32, queue_size=1)
		self.marker_scan_command_pub = rospy.Publisher("/edrone/marker_scan_cmd", Bool, queue_size=1)

		self.mission_status_pub = rospy.Publisher("/edrone/mission_status", Bool, queue_size=1)

		# Subscribing to /edrone/gps, /edrone/gripper_check, /destination_coordinates, /qr_status, /edrone/range_finder_top
		rospy.Subscriber("/edrone/gps", NavSatFix, self.gps_callback)
		rospy.Subscriber("/destination_coordinates", Vector3, self.destination_coordinates_callback)
		rospy.Subscriber("/edrone/gripper_check", String, self.gripper_check_callback)
		rospy.Subscriber("/qr_status", Bool, self.qr_status_callback)
		#rospy.Subscriber("/pid_tuning_altitude", PidTune, self.longitude_set_pid)
		rospy.Subscriber("/edrone/range_finder_top", LaserScan, self.range_finder_top_callback, queue_size=1)
		rospy.Subscriber("/edrone/range_finder_bottom", LaserScan, self.range_finder_bottom_callback)
		rospy.Subscriber("/edrone/marker_visibility", Bool, self.marker_visibility_callback)
		rospy.Subscriber("/edrone/err_x_m", Float32, self.err_x_m_callback)
		rospy.Subscriber("/edrone/err_y_m", Float32, self.err_y_m_callback)

		rospy.Subscriber("/edrone/destination", NavSatFix, self.destination_callback)
		rospy.Subscriber("/edrone/destination_cmd", String, self.destination_cmd_callback)
		rospy.Subscriber("/edrone/building_id", Int32, self.building_id_callback)


		# to turn on the drone
		self.cmd_pub.publish(self.control_cmd)

		# to delay time in the location for accuracy
		self.arival_time = 0


	# destination coordinates callback
	def destination_callback(self, msg):
		self.destination[0] = msg.latitude
		self.destination[1] = msg.longitude
		self.destination[2] = msg.altitude
		#print(self.destination)


	# destination coordinates cmd callback
	def destination_cmd_callback(self, msg):
		self.destination_cmd = msg.data
		#print(self.destination_cmd)

	# building id callback
	def building_id_callback(self, msg):
		#if self.destination_cmd == "Drop" and self.building_id.data != msg.data:
		#	self.update_destination = False
		
		self.building_id.data = msg.data


	def err_x_m_callback(self, msg):
		self.err_x_m = msg.data
		self.err_x = msg.data / 110692.0702932625
		#print(self.err_x_m)
		if self.marker_scan.data and ((self.error[0] > -0.000004517 and self.error[0] < 0.000004517) and (self.error[1] > -0.0000047487 and self.error[1] < 0.0000047487)) and self.destination_cmd == "Drop" and self.search_circle:
			self.search_circle[self.search_index][0] += self.err_x

	def err_y_m_callback(self, msg):
		self.err_y_m = msg.data
		self.err_y_m -= 0.35
		self.err_y = self.err_y_m / (105292.0089353767)
		#print(self.err_x_m)
		if self.marker_scan.data and ((self.error[0] > -0.000004517 and self.error[0] < 0.000004517) and (self.error[1] > -0.0000047487 and self.error[1] < 0.0000047487)) and self.destination_cmd == "Drop" and self.search_circle:
			self.search_circle[self.search_index][1] += self.err_y

	# marker coordinates callback function
	def marker_coordinates_callback(self, msg):
		self.marker_coordinates[0] = msg.data[0]
		self.marker_coordinates[1] = msg.data[1]
		self.marker_coordinates[2] = msg.data[2]
		self.marker_coordinates[3] = msg.data[3]

		

		#print(self.centre_x_pixel , self.centre_y_pixel)
		print([self.err_x_m, self.err_y_m])

	def marker_visibility_callback(self, msg):
		self.marker_visibility = msg.data

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

		if self.Z_m:
			self.Z_m.data = float(self.drone_position[-1] - deepcopy(self.destination)[-1])
			if self.destination_cmd == "Drop":
				self.Z_m.data -= 1

		# print(self.drone_position)

	
	# range finder top callback function
	def range_finder_top_callback(self, msg):
		for i in range(4):
			if msg.ranges[i] > 0.7:
				self.distances[i] = msg.ranges[i]
				self.distances[i] = float(self.distances[i])
	
	# range finder bottom callback function
	def range_finder_bottom_callback(self, msg):
			self.distances[4] = msg.ranges[0] - 0.2
			self.distances[4] = float(self.distances[4])
			#print(self.distances[4])

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
		self.gripper_success = False
		while self.package_pickable and not self.gripper_success:
			self.gripper_success = self.gripper_service(True)

		if self.gripper_success:
			self.package_picked = True
			self.mission_status.data = True



	# function to pick package up
	def drop_package(self):
		if self.package_picked:
			self.gripper_success = self.gripper_service(False)
			self.package_picked = False

			#self.safe_pos2 = deepcopy(self.drone_position)
			#self.safe_pos2[-1] += 4


	def set_waypiont(self):

		self.mission_status.data = False

		if (
			(self.error_check[0] > -0.000004517 and self.error_check[0] < 0.000004517)
			and (self.error_check[1] > -0.0000047487 and self.error_check[1] < 0.0000047487)
			and (self.error_check[2] > -0.2 and self.error_check[2] < 0.2)
		):
			
			if self.destination_cmd == "Pickup":
				#print("Pickup")
				self.pick_package()


			elif self.destination_cmd == "Sleep":

				self.control_cmd.aux1 = 1000
		

			elif self.destination_cmd == "Drop":
				# start scanning
				self.marker_scan.data = True


			if self.destination_cmd == "Reach": 
				self.mission_status.data = True

		elif self.destination_cmd == "Drop" and self.marker_scan.data:

				# when hovering above the marker
				if self.marker_visibility and (self.err_x_m > -0.2 and self.err_x_m < 0.2) and (self.err_y_m > -0.2 and self.err_y_m < 0.2):
					if self.Z_m.data < 1:

						self.drop_package()
						self.mission_status.data = True
						#print("dropped")

						self.mission_status_pub.publish(self.mission_status)
						self.update_destination = True


		if self.destination_cmd == "Drop" and self.update_destination:
			self.mission_status.data = True
			self.mission_status_pub.publish(self.mission_status)

		else: 
			self.update_destination = False

						
			'''else:
																	self.marker_scan.data = True'''
				



		self.mission_status_pub.publish(self.mission_status)




				# to move to next location
		# checking tolerance in lat, long, alt respectively
		'''     if (
			(self.error_check[0] > -0.000004517 and self.error_check[0] < 0.000004517)
			and (self.error_check[1] > -0.0000047487 and self.error_check[1] < 0.0000047487)
			and (self.error_check[2] > -0.2 and self.error_check[2] < 0.2)
		):

			if not self.arival_time:
				self.arival_time = rospy.Time.now().to_sec()

			elif rospy.Time.now().to_sec() - self.arival_time > 1 and (
				(self.error_check[0] > -0.000004517 and self.error_check[0] < 0.000004517)
				and (self.error_check[1] > -0.0000047487 and self.error_check[1] < 0.0000047487)
				and (self.error_check[2] > -0.2 and self.error_check[2] < 0.2)
			):

				self.arival_time = 0
				print("Location {} reached.".format(self.location_index + 1))

				if not self.location_index == 2 * self.building_id.data:

					#print("Location {} reached.".format(self.location_index + 1))
					self.marker_scan.data = False
					if not self.location_index == len(self.location_setpoints) -1:
						self.location_index += 1




				if not self.location_index == 2 * self.building_id.data and self.location_index == len(self.location_setpoints) - 1 and self.control_cmd.aux1 == 2000 and self.distances[-1] < 1:
					self.control_cmd.aux1 = 1000

					print("Final location reached.")

		if self.location_index == 2 * self.building_id.data:
			#print(self.Z_m, self.distances[-1])

			if self.marker_visibility and (self.err_x_m > -0.2 and self.err_x_m < 0.2) and (self.err_y_m > -0.2 and self.err_y_m < 0.2):
				self.marker_scan.data = False
				if not self.location_index == len(self.location_setpoints) -1:
					self.location_index += 1
					print("hi")
			else:
				self.marker_scan.data = True

			if self.location_index == len(self.location_setpoints) -1 and self.marker_visibility and (self.err_x_m > -0.2 and self.err_x_m < 0.2) and (self.err_y_m > -0.2 and self.err_y_m < 0.2):
					self.location_setpoints.append(deepcopy(self.drone_position))

		self.building_id.data = ((self.location_index + 1) / 2)
		#print(self.building_id)
		if self.building_id.data < 1:
			self.building_id.data = 1
		if self.building_id.data > len(self.building_coordinates):
			self.building_id.data = len(self.building_coordinates)'''

					#print("Tolerance:")
					#print("lat = {}".format(self.error[0]))
					#print("long = {}".format(self.error[1]))
					#print("alt = {}".format(self.error[2]))
		#print(self.location_index)
		#print((self.err_x_m > -0.2 and self.err_x_m < 0.2) and (self.err_y_m > -0.2 and self.err_y_m < 0.2))

	# to check for an obstacle
	def obstacle_encountered(self):
		
		if self.distances[0] <= self.detection_distance or self.distances[1] <= self.detection_distance or self.distances[2] <= self.detection_distance or self.distances[3] <= self.detection_distance:
			if self.distances[0] <= self.safe_distance or self.distances[1] <= self.safe_distance or self.distances[2] <= self.safe_distance or self.distances[3] <= self.safe_distance:
				if not self.encounter_time and (self.distances[4] >= 2.5 or self.package_picked):
					self.safe_pos = deepcopy(self.drone_position)

					# distance[1]
					if self.distances[1] < self.safe_distance and (self.distances[-1] > 2.5 or self.package_picked) and (self.error_check[0] > 0) and (self.error_check[2] > -0.2 and self.error_check[2] < 0.2):
						print(True)
						self.safe_pos2 = deepcopy(self.drone_position)
						self.safe_pos2[0] += self.metres_to_lat(6)
						self.safe_pos2[1] -= self.meters_to_long(6)

					#print.("safe pos", self.safe_pos)
					self.encounter_time = rospy.Time.now().to_sec()
					#print("hello")

				#if rospy.Time.now().to_sec() - self.encounter_time > 4:
					#self.safe_pos = []
					#self.encounter_time = 0
			
			#print("obstacle encountered")
			return True
		else:
			if (not self.encounter_time):
				self.safe_pos = None
				# print("safe pos deleted")
				self.encounter_time = 0
				#print("bye")
			elif rospy.Time.now().to_sec() - self.encounter_time > 1:
				self.safe_pos = None
				self.encounter_time = 0
				#print("bye")

			#print("bye")
			
		return False

	def metres_to_lat(self, lat):
		return (lat / 110692.0702932625)

	def meters_to_long(self, lon):
		return (lon / -105292.0089353767)

	def bug0_crawl(self):
		if self.obstacle_encountered():
			if self.distances[0] < self.safe_distance or self.distances[1] < self.safe_distance or self.distances[2] < self.safe_distance or self.distances[3] < self.safe_distance:
				if self.encounter_time:
					if rospy.Time.now().to_sec() - self.encounter_time > 2 or True:
						
						# checking direction of the obstacke and updating the next position accordingly
						# along latitude
						if self.distances[3] < self.safe_distance or self.distances[1] < self.safe_distance:
							self.error[0] = self.safe_pos[0] - self.drone_position[0]
							#print("crawling like a bug")
							
							#if self.error[1] > -0.0000047487 * 2 and self.error[1] < 0.0000047487 * 2:
							#	self.safe_pos[2] += 8
						# along longitude
						if self.distances[0] < self.safe_distance or self.distances[2] < self.safe_distance:
							self.error[1] = self.safe_pos[1] - self.drone_position[1]
							#print("crawling like a bug")
							
							#if self.error[0] > -0.000004517 * 2 and self.error[0] < 0.000004517 * 2:
							#	self.safe_pos[2] += 8
							#print(self.distances[0])

						self.error[2] = self.safe_pos[2] - self.drone_position[2]
							


						#self.safe_pos[1] += 0.0000015
					#for i in range(3):
						#self.error[i] = self.safe_pos[i] - self.drone_position[i]
					#print("bug")
					#self.error[2] = self.safe_pos[2] - self.drone_position[2]
				#else:
					#print("not")



	def cmd(self):

		if not self.destination_cmd == "Drop":
			self.marker_scan.data = False

		if True:
			#print(self.safe_pos)

			# updating coordinates according to the new marker position
			if self.marker_scan.data and self.destination_cmd == "Drop":

				if not self.search_circle:
					lat = self.destination[0]
					lon = self.destination[1]
					alt = self.destination[2] + 10
					self.search_circle = [[lat,lon, alt + 1], [lat - 0.00006, lon + 0.00006, alt], [lat - 0.00006, lon - 0.00006, alt], [lat + 0.00006, lon - 0.00006, alt], [lat + 0.00006, lon + 0.00006, alt]]
					#self.search_circle = [[lat - 0.00006, lon - 0.00006, alt], [lat + 0.00006, lon - 0.00006, alt], [lat + 0.00006, lon + 0.00006, alt]]
					self.search_index = 0

					#self.destination[2] += 0.08
					self.time = None
				
				elif (self.err_x_m > -0.2 and self.err_x_m < 0.2) and (self.err_y_m > -0.2 and self.err_y_m < 0.2):
					if not self.time:
						self.time = rospy.Time.now().to_sec()
					elif rospy.Time.now().to_sec() - self.time > 0.4:
						if self.Z_m.data > 1:
							self.search_circle[self.search_index][2] -= 1
				
						self.time = rospy.Time.now().to_sec()
				

			else:
				self.search_circle = None
				self.time = None

			'''if not self.marker_scan.data:
				if self.distances[-1] > 0.64:
					self.destination[-1] -= 0.1'''

			




			# Computing error (for proportional)


			for i in range(3):
				self.error[i] = (
						self.destination[i] - self.drone_position[i]
					)
				self.error_check[i] = (
						self.destination[i] - self.drone_position[i]
					)
				
				if self.marker_scan.data and self.search_circle:
					#print(self.search_index)
					self.error[i] = (
						self.search_circle[self.search_index][i] - self.drone_position[i]
					)
				if self.safe_pos2:
					self.error[i] = self.safe_pos2[i] - self.drone_position[i]

				if self.safe_pos2 and ((self.error[0] > -0.000004517 and self.error[0] < 0.000004517) and (self.error[1] > -0.0000047487 and self.error[1] < 0.0000047487) and(self.error[2] > -0.2 and self.error[2] < 0.2)):
					self.safe_pos2 = None
					print("safe_pos2 removed")


					

			if ((self.error[0] > -0.000004517 and self.error[0] < 0.000004517) and (self.error[1] > -0.0000047487 and self.error[1] < 0.0000047487) and(self.error[2] > -0.2 and self.error[2] < 0.2)) and not self.marker_visibility:
				if not self.timeout:
					self.timeout = rospy.Time.now().to_sec()
				if self.search_index < 4 and rospy.Time.now().to_sec() - self.timeout > 1:
					self.search_index += 1
					self.timeout = None

			if self.obstacle_encountered():
				self.bug0_crawl()


			#   self.p_error_limit = 0.00002
			#else:
			#   self.p_error_limit = 0.00002


			if (self.error[2] > -0.2 and self.error[2] < 0.2):
				self.p_error_limit = [10/110692.0702932625, 10/105292.0089353767, 1.4]

			if (not ((self.error[0] > -0.000004517 and self.error[0] < 0.000004517) or (self.error[1] > -0.0000047487 and self.error[1] < 0.0000047487))) or self.obstacle_encountered() or (self.distances[4] < 4 if not self.package_picked else False):
				
				self.p_error_limit = [2.5/110692.0702932625, 2.5/105292.0089353767, 1.4]

			elif self.marker_scan.data and self.destination_cmd == "Drop" and self.marker_visibility:
				self.p_error_limit = [2.5/110692.0702932625, 2.5/105292.0089353767, 1.4]

			for i in range(3):

				if i != 2:
					if self.error[i] > self.p_error_limit[i]:
						self.p_error[i] = self.p_error_limit[i]
					elif self.error[i] < -self.p_error_limit[i]:
						self.p_error[i] = -self.p_error_limit[i]
					else:
						self.p_error[i] = self.error[i]
						#print(type(self.error[i]))
				else:

					# to hover above a distance above the ground 
					if self.distances[4] < 4 and not self.package_picked and not ((self.err_x_m > -0.2 and self.err_x_m < 0.2) and (self.err_y_m > -0.2 and self.err_y_m < 0.2) if self.destination_cmd == "Drop" else False):
						diff = 4 - self.distances[4]
						if self.error[i] <= diff and not ((self.error[0] > -0.00001 and self.error[0] < 0.00001) and (self.error[1] > -0.00001 and self.error[1] < 0.00001)):
							self.error[i] = diff
						
						#print(type(self.error[i])),

					if self.error[i] > self.p_error_limit[i]:
						self.p_error[i] = self.p_error_limit[i]
					elif self.error[i] < -self.p_error_limit[i]:
						self.p_error[i] = -self.p_error_limit[i]
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

			#print(self.p_error)

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

			# print(self.control_cmd.rcThrottle)

			# Update previous error value
			for i in range(3):
				self.prev_value[i] = self.error[i]

			# print(self.control_cmd.rcRoll)
			# print(self.control_cmd.rcPitch)

			self.cmd_pub.publish(self.control_cmd)
			self.altitude_pub.publish(self.error[0])

			self.zero_pub.publish(0.0)

			self.qr_command_pub.publish(self.qr_command)

			self.set_waypiont()

			self.Z_m_error_pub.publish(self.Z_m)
			self.marker_scan_command_pub.publish(self.marker_scan)
			#print(self.Z_m)



if __name__ == "__main__":
	Controller = Control()
	r = rospy.Rate(20)
	while not rospy.is_shutdown():
		Controller.cmd()
		r.sleep()
