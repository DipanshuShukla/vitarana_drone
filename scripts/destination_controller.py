#!/usr/bin/env python

# ros node for setting the destination coordinates

import rospy
from copy import deepcopy
from sensor_msgs.msg import NavSatFix

from std_msgs.msg import Float32, Bool, String, Int32

# change filename as needed
FILE_NAME = "/home/codebunny/catkin_ws/src/vitarana_drone/scripts/manifest.csv"


class destination():
	def __init__(self, csv_file):
		
		rospy.init_node('destination_controller') #Initialise rosnode 

		self.destination_list = [] # contains [lat, lon, alt, cmd]
		self.destination_index = 0

		# to delay next waypoint
		self.reached = None

		self.csv_file = csv_file


		# to calculate box location
		self.A1_pos = [18.9999864489, 71.9999430161, 8.44099749139] # lat, lon, alt
		self.cell_size = [self.m_to_lat(1.5), self.m_to_lon(1.5)] # lat, lon


		

		self.building_id = Int32()
		self.building_id.data = 1

		self.init_pos = None


		print("Data loaded...")


		# to check if the drone completed the current mission
		self.mission_success = False

		# publishers
		self.destination_pub = rospy.Publisher("/edrone/destination", NavSatFix, queue_size=1)
		# to tell the drone what to do at the destinAtion
		self.destination_cmd_pub = rospy.Publisher("/edrone/destination_cmd", String, queue_size=1)

		self.building_id_pub = rospy.Publisher("/edrone/building_id", Int32, queue_size=1)


		# subscribers
		rospy.Subscriber("/edrone/mission_status", Bool, self.mission_status_callback)
		rospy.Subscriber("/edrone/gps", NavSatFix, self.gps_callback)

		# for data to be pu blished

		

	def gps_callback(self, msg):
		if not self.init_pos:
			self.init_pos = [0.0, 0.0, 0.0]
			self.init_pos[0] = msg.latitude
			self.init_pos[1] = msg.longitude
			self.init_pos[2] = msg.altitude

			first_pos = deepcopy(self.init_pos)
			first_pos[-1] += 4
			first_pos.append("Reach")

			self.destination_list.append(first_pos)

			# reading csv file and populating the destination array
			print("Loading data...")
			f= open(self.csv_file, "r")

			for line in f:

				items = line.split(",")

				pickup_zone = self.cell_to_coordinates(items[0])
				pickup_zone_copy = deepcopy(pickup_zone)
				pickup_zone_copy[-2] += 4
				pickup_zone_copy[-1] = "Reach"

				items = [float(item) for item in items[1:]]
				items[-1] += 1 
				items.append("Drop")
				items_copy = deepcopy(items)
				
				items_copy[-2] += 4
				items_copy[-1] = "Reach"

				self.destination_list.append(pickup_zone_copy)
				self.destination_list.append(pickup_zone)
				self.destination_list.append(pickup_zone_copy)

				self.destination_list.append(items_copy)
				self.destination_list.append(items)
				#self.destination_list.append(items_copy)

			# final pos
			self.init_pos.append("Sleep")

			init_pos2 = deepcopy(first_pos)

			self.destination_list.append(init_pos2)

			self.destination_list.append(self.init_pos)

			self.destination = NavSatFix()
			self.destination.latitude = self.destination_list[-3][0]
			self.destination.longitude = self.destination_list[-3][1]
			self.destination.altitude = self.destination_list[-3][2]

			self.destination_cmd = String()
			self.destination_cmd.data = self.destination_list[-3][3]

			for destination in self.destination_list:
				print(destination)

			print(self.destination_list[self.destination_index][3])



	# mission_status_callback
	def mission_status_callback(self, msg):
		self.mission_success = msg.data

	
	# to convert distance in meters to latitute
	def m_to_lat(self, m):
		return m / 110692.0702932625

	
	# to convert distance in meters to longitute
	def m_to_lon(self, m):
		return m / 105292.0089353767


	# convert cell ID to lat, lon, alt
	def cell_to_coordinates(self, cell_ID):
		
		cells = "ABC"

		lat = self.A1_pos[0] + self.cell_size[0] * cells.index(cell_ID[0])
		lon = self.A1_pos[1] + self.cell_size[1] * (int(cell_ID[1]) - 1)
		alt = self.A1_pos[2]

		return [lat, lon, alt, "Pickup"]



	#to set destionation 
	def set_destination(self):

		if self.init_pos:

			#in case of mission success update mission
			if self.mission_success:
																		
						#print("mission_success")

				if not self.reached:
					self.reached = rospy.Time.now().to_sec()
				else:

					if not self.destination_index >= len(self.destination_list) - 1 and (rospy.Time.now().to_sec() - self.reached > 1):
						self.destination_index += 1
						self.reached = None
						
						if self.destination_cmd.data == "Drop":
							self.building_id.data += 1

						self.destination.latitude = self.destination_list[self.destination_index][0]
						self.destination.longitude = self.destination_list[self.destination_index][1]
						self.destination.altitude = self.destination_list[self.destination_index][2]

						self.destination_cmd.data = self.destination_list[self.destination_index][3]

						
						print(self.destination_cmd, self.building_id.data)

				


			# setting mission parameters
			self.destination.latitude = self.destination_list[self.destination_index][0]
			self.destination.longitude = self.destination_list[self.destination_index][1]
			self.destination.altitude = self.destination_list[self.destination_index][2]

			self.destination_cmd.data = self.destination_list[self.destination_index][3]


			# publishing mission parameters
			self.destination_pub.publish(self.destination)
			self.destination_cmd_pub.publish(self.destination_cmd)
			self.building_id_pub.publish(self.building_id)
		




if __name__ == '__main__':
	destination_manager = destination(csv_file=FILE_NAME)
	r = rospy.Rate(20)
	
	# running appropriate function continuously in loop
	while not rospy.is_shutdown():
		destination_manager.set_destination()
		r.sleep()