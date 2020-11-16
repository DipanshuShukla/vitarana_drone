#!/usr/bin/env python

# 			PUBLICATIONS				SUBSCRIPTIONS
# 		/edrone/drone_command			/edrone/gps
# 		/altitude_error					/pid_tuning_altitude
# 		/zero_error                     /edrone/range_finder_top
# 		/latitude_error                 /edrone/gripper_check
# 		/longitude_error


from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import NavSatFix, LaserScan
from std_msgs.msg import Float32, Bool
import rospy
import time


class Control:
    def __init__(self):
        rospy.init_node(
            "position_controller"
        )  # initializing ros node with name drone_control

        self.drone_position = [0.0, 0.0, 0.0]  # [lat, long, alt]

        self.location_setpoints = [[19.0009248718, 71.9998318945, 22.16 + 2]]  # [lat, long, alt]
        #self.location_setpoints = []

        self.box_location = [19.0007046575, 71.9998955286, 22.1599967919]
        #self.box_location = [19.0, 72.0, 0.31]

        self.location_setpoints.append([self.box_location[0], self.location_setpoints[0][1], self.location_setpoints[0][2]])

        self.location_setpoints.append([self.box_location[0], self.box_location[1], self.box_location[2] + 2])
        self.location_setpoints.append([self.box_location[0], self.box_location[1], self.box_location[2] + 0.6])
        self.location_setpoints.append(self.box_location)

        #print(self.location_setpoints)

        self.location_index = 0

        
        # for bug0 algorithm
        self.distances = [0.0, 0.0, 0.0, 0.0] # [front, right, rear, left]
        self.obstacle_encountered = False

        # for scanning the qrcode and picking the package
        self.scan_qr = False
        self.qr_scanned = False


        self.package_pickable = False

        
        time.sleep(2.5)

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
        self.Ki = [0.2, 0.2, 0.002]
        self.Kd = [20000000, 20000000, 4000.0]

        self.prev_value = [0.0, 0.0, 0.0]  # [lat, long, alt]
        self.max_value = 2000
        self.min_value = 1000

        self.error = [0.0, 0.0, 0.0]  # [lat, long, alt]
        self.p_error = [0.0, 0.0, 0.0]  # [lat, long, alt]
        self.dif_error = [0.0, 0.0, 0.0]
        self.sum_error = [0.0, 0.0, 0.0]  # Iterm

        self.output = [0.0, 0.0, 0.0]  # [lat, long, alt]

        self.p_error_limit = 0.00008

        # Publishing /edrone/drone_command, /altitude_error, /zero_error
        self.cmd_pub = rospy.Publisher("/drone_command", edrone_cmd, queue_size=1)
        self.altitude_pub = rospy.Publisher("/altitude_error", Float32, queue_size=1)
        self.zero_pub = rospy.Publisher("/zero_error", Float32, queue_size=1)

        # Subscribing to /edrone/gps, /edrone/range_finder_top
        rospy.Subscriber("/edrone/gps", NavSatFix, self.gps_callback)
        rospy.Subscriber("/edrone/range_finder_top", LaserScan, self.range_finder_callback)
        rospy.Subscriber("/edrone/range_finder_top", Bool, self.gripper_check_callback)
        # rospy.Subscriber("/pid_tuning_altitude", PidTune, self.altitude_set_pid)

        # to turn on the drone
        self.cmd_pub.publish(self.control_cmd)

        # to delay time in the location for accuracy
        self.arival_time = 0


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
        #self.package_pickable = msg.data
        pass
    
    def latitude_set_pid(self, latitude):
        self.Kp[0] = latitude.Kp * 500
        self.Ki[0] = latitude.Ki * 0.004
        self.Kd[0] = latitude.Kd * 8000

    def longitude_set_pid(self, longitude):
        self.Kp[1] = longitude.Kp * 500
        self.Ki[1] = longitude.Ki * 0.004
        self.Kd[1] = longitude.Kd * 8000

    def altitude_set_pid(self, altitude):
        self.Kp[2] = altitude.Kp * 0.4
        self.Ki[2] = altitude.Ki * 0.002
        self.Kd[2] = altitude.Kd * 0.8

    def scan_qr_code(self):
        print("scanning qr...")
        # TODO
        pass

    def drop_package(self):
        # TODO
        pass

    # to check for an obstacle
    def obstacle_encounter(self):
        # TODO
        return False

    def bug0_crawl(self):
        # TODO
        pass



    def cmd(self):

        # Computing error (for proportional)
        for i in range(3):

            if not self.obstacle_encounter():
                self.error[i] = (
                    self.location_setpoints[self.location_index][i] - self.drone_position[i]
                )
            else:
                self.bug0_crawl()

            if i != 2:
                if self.error[i] > self.p_error_limit:
                    self.p_error[i] = self.p_error_limit
                elif self.error[i] < -self.p_error_limit:
                    self.p_error[i] = -self.p_error_limit
                else:
                    self.p_error[i] = self.error[i]
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

        # print(self.control_cmd.rcThrottle)

        # Update previous error value
        for i in range(3):
            self.prev_value[i] = self.error[i]

        # print(self.control_cmd.rcRoll)
        # print(self.control_cmd.rcPitch)

        self.cmd_pub.publish(self.control_cmd)
        self.altitude_pub.publish(self.error[2])

        self.zero_pub.publish(0.0)

        # to move to next location
        # checking tolerance in lat, long, alt respectively
        if (
            (self.error[0] > -0.000004517 and self.error[0] < 0.000004517)
            and (self.error[1] > -0.0000047487 and self.error[1] < 0.0000047487)
            and (self.error[2] > -0.2 and self.error[2] < 0.2)
        ):

            if not self.arival_time:
                self.arival_time = rospy.Time.now().to_sec()

            elif rospy.Time.now().to_sec() - self.arival_time > 1 and (
                (self.error[0] > -0.000004517 and self.error[0] < 0.000004517)
                and (self.error[1] > -0.0000047487 and self.error[1] < 0.0000047487)
                and (self.error[2] > -0.2 and self.error[2] < 0.2)
            ):

                self.arival_time = 0


                if self.location_index == len(self.location_setpoints) - 2:
                    
                    if self.qr_scanned:
                        self.location_index += 1
                    else:
                        self.scan_qr_code()


                elif not self.location_index == len(self.location_setpoints) - 1 or not self.location_index == len(self.location_setpoints) - 2:

                    print("Location {} reached.".format(self.location_index + 1))

                    self.location_index += 1

                elif self.control_cmd.aux1 == 2000:

                    self.control_cmd.aux1 = 1000

                    print("Final location reached.")

                    #print("Tolerance:")
                    #print("lat = {}".format(self.error[0]))
                    #print("long = {}".format(self.error[1]))
                    #print("alt = {}".format(self.error[2]))


if __name__ == "__main__":
    Controller = Control()
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        Controller.cmd()
        r.sleep()
