#!/usr/bin/env python

# ros node for setting the destination coordinates

import rospy

from sensor_msgs.msg import NavSatFix

from std_msgs.msg import Float32, Bool, String

# change filename as needed
FILE_NAME = "/home/codebunny/catkin_ws/src/vitarana_drone/scripts/manifest.csv"


class destination():
	def __init__(self, csv_file):
		
		rospy.init_node('barcode_test') #Initialise rosnode 

		self.destination_list = [] # contains [lat, lon, alt, cmd]
		self.destination_index = 0

		# to delay next waypoint
		self.reached = None


		# to calculate box location
		self.A1_pos = [18.9999864489, 71.9999430161, 8.44099749139] # lat, lon, alt
		self.cell_size = [self.m_to_lat(1.5), self.m_to_lon(1.5)] # lat, lon


		# reading csv file and populating the destination array
		f= open(csv_file, "r")

		for line in f:
			items = line.split(",")

			self.destination_list.append(self.cell_to_coordinates(items[0]))
			self.destination_list.append([float(items[1]), float(items[2]), float(items[3]), "Drop"])



		# to check if the drone completed the current mission
		self.mission_success = False

		# publishers
		self.destination_pub = rospy.Publisher("/edrone/destination", NavSatFix, queue_size=1)
		# to tell the drone what to do at the destinAtion
		self.destination_cmd_pub = rospy.Publisher("/edrone/destination_cmd", String, queue_size=1)


		# subscribers
		rospy.Subscriber("/edrone/mission_status", Bool, self.mission_status_callback)

		# for data to be published

		self.destination = NavSatFix()
		self.destination.latitude = self.destination_list[0][0]
		self.destination.longitude = self.destination_list[0][1]
		self.destination.altitude = self.destination_list[0][2]

		self.destination_cmd = String()
		self.destination_cmd.data = self.destination_list[0][3]

		for destination in self.destination_list:
			print(destination)



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

		#in case of mission success update mission
		if self.mission_success:
			if not self.reached:
				self.reached = rospy.Time.now().to_sec()
			else:

				if not self.destination_index >= len(self.destination_list) - 1 and rospy.Time.now().to_sec() - self.reached > 0.5:
					self.destination_index += 1
					self.reached = None

		# setting mission parameters
		self.destination.latitude = self.destination_list[self.destination_index][0]
		self.destination.longitude = self.destination_list[self.destination_index][1]
		self.destination.altitude = self.destination_list[self.destination_index][2]

		self.destination_cmd.data = self.destination_list[self.destination_index][3]


		# publishing mission parameters
		self.destination_pub.publish(self.destination)
		self.destination_cmd_pub.publish(self.destination_cmd)




if __name__ == '__main__':
	destination_manager = destination(csv_file=FILE_NAME)
	r = rospy.Rate(5)
	
	# running appropriate function continuously in loop
	while not rospy.is_shutdown():
		destination_manager.set_destination()
		r.sleep()