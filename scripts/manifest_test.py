import math
from copy import deepcopy
FILE_NAME = "/home/codebunny/catkin_ws/src/vitarana_drone/scripts/manifest.csv"

# data structure for storing missions
class mission:
	def __init__(self):
		self.start = None
		self.end = None
		self.distance = 0.0
		self.objective = None

	def __repr__(self):
		return str(self.distance) + " , " + str(self.objective) + " , " + str(self.start) + " , " + str(self.end)

class mission_planner:
	def __init__(self):
		self.mission_list = []

		# to calculate box location
		self.A1_pos = [18.9998102845, 72.000142461, 16.757981] # lat, lon, alt
		self.X1_pos = [18.9999367615, 72.000142461, 16.757981] # lat, lon, alt
		self.cell_size = [self.m_to_lat(1.5), self.m_to_lon(1.5)] # lat, lon
		self.delivery_grid = "ABC"
		self.return_grid = "XYZ"

		self.destination_list = []

	# to convert distance in meters to latitute
	def m_to_lat(self, m):
		return m / 110692.0702932625

	
	# to convert distance in meters to longitute
	def m_to_lon(self, m):
		return m / 105292.0089353767

	# to convert distance in latitute to meters
	def lat_to_m(self, m):
		return m * 110692.0702932625

	
	# to convert distance in longitute to meters
	def lon_to_m(self, m):
		return m * 105292.0089353767

	def get_destination_list(self):
		if not self.destination_list:
			for item in self.mission_list:

				start = deepcopy(item.start)
				start1 = deepcopy(start)
				start1[-1] +=6
				start1.append("Reach")
				start.append("Pickup")

				end = deepcopy(item.end)
				end1 = deepcopy(end)
				end1[-1] +=6
				end1.append("Reach")
				end.append("Drop" if not item.objective == "RETURN" else "Return")

				for destination in [start1, start, end1, end]:
					self.destination_list.append(destination)

		return self.destination_list

	def sort(self):
		self.mission_list = self.sorted(self.mission_list)


	# merge sort the missions according to the linear distances
	def sorted(self, mission_list):

		size = len(mission_list)
		
		if size <= 1:
			return deepcopy(mission_list)

		middle = size // 2

		left = self.sorted(mission_list[:middle])

		right = self.sorted(mission_list[middle:])

		new_list = []

		i = 0
		j = 0

		while i < len(left) and j < len(right):
			if left[i].distance > right[j].distance:
				new_list.append(left[i])
				i += 1
			else:
				new_list.append(right[j])
				j += 1

		# Checking if any element was left
		while i < len(left):
			new_list.append(left[i])
			i += 1
 
		while j < len(right):
			new_list.append(right[j])
			j += 1
			

		'''for item in new_list:
									print(item)
						
								print("\n--\n")'''

		return deepcopy(new_list)

	def use_timestone(self):
		return True


	def dr_strange(self):
		return self.use_timestone()


	def cell_to_coordinates(self, cell_ID, delivery):
		
		cells = self.delivery_grid if delivery else self.return_grid
		pos = self.A1_pos if delivery else self.X1_pos
		

		lat = pos[0] + self.cell_size[0] * cells.index(cell_ID[0])
		lon = pos[1] + self.cell_size[1] * (int(cell_ID[1]) - 1)
		alt = pos[2]

		return [lat, lon, alt]



	def add(self, objective, start, end):
		# item is [objective, start, end]
		m = mission()
		m.objective = objective
		m.start = deepcopy(start)
		m.end = deepcopy(end)

		# calculating linear distance
		# d**2 = (x2 - x1)**2 + (y2 - y1)**2
		m.distance = math.sqrt((self.lat_to_m(end[0] - start[0]))**2 + (self.lon_to_m(end[1] - start[1]))**2)

		self.mission_list.append(m)


	def plan_mission(self):
		pass

mp = mission_planner()

print("Loading data...\n")
f = open(FILE_NAME, "r")

for line in f:

	items = line.replace('\n', '').replace(' ', '').split(",")

	objective = items[0]

	start = [float(x) for x in items[1].split(";")] if len(items[1]) != 2 else mp.cell_to_coordinates(items[1], True)

	end = [float(x) for x in items[2].split(";")] if len(items[2]) != 2 else mp.cell_to_coordinates(items[2], False)

	#print([objective,start,end])

	mp.add(objective, start, end)
	
	#break

for m in mp.mission_list:
	print(m)

print("\nNo. of missions = " + str(len(mp.mission_list)))


print("\nData loaded.\n")

print("Sorting according to linear distance...\n")

mp.sort()
print("Sorted.\n")

for m in mp.mission_list:
	print(m)


print("\nNo. of missions = " + str(len(mp.mission_list)))

print("Calculating destinations map...")

print("Done.\n")

destinations = mp.get_destination_list()

for destination in destinations:
	print(destination)

print("\n")

print(mp.dr_strange())