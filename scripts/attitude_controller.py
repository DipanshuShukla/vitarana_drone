#!/usr/bin/env python

"""
This python file runs a ROS-node of name attitude_control which controls the roll pitch and yaw angles of the eDrone.
This node publishes and subsribes the following topics:
		PUBLICATIONS            SUBSCRIPTIONS
		/roll_error             /pid_tuning_altitude
		/pitch_error            /pid_tuning_pitch
		/yaw_error              /pid_tuning_roll
		/edrone/pwm             /edrone/imu/data
								/edrone/drone_command

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.
"""

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import rospy
import time
import tf, math


class Edrone:
    """docstring for Edrone"""

    def __init__(self):
        rospy.init_node(
            "attitude_controller"
        )  # initializing ros node with name drone_control

        # This corresponds to your current orientation of eDrone in quaternion format. This value must be updated each time in your imu callback
        # [x,y,z,w]
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        # This corresponds to your current orientation of eDrone converted in euler angles form.
        # [r,p,y]
        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        # This is the setpoint that will be received from the drone_command in the range from 1000 to 2000
        # [r_setpoint, p_setpoint, y_setpoint]
        self.setpoint_cmd = [0.0, 0.0, 0.0]

        # aux1 to enable/disable drone
        self.aux1 = 1000

        # The setpoint of orientation in euler angles at which you want to stabilize the drone
        # [r_setpoint, p_psetpoint, y_setpoint]
        self.setpoint_euler = [0.0, 0.0, 0.0]

        self.error_check = [0.0, 0.0, 0.0]

        # Declaring pwm_cmd of message type prop_speed and initializing values
        # Hint: To see the message structure of prop_speed type the following command in the terminal
        # rosmsg show vitarana_drone/prop_speed

        self.pwm_cmd = prop_speed()
        self.pwm_cmd.prop1 = 0.0
        self.pwm_cmd.prop2 = 0.0
        self.pwm_cmd.prop3 = 0.0
        self.pwm_cmd.prop4 = 0.0

        # initial setting of Kp, Kd and Ki for [roll, pitch, yaw]. eg: self.Kp[2] corresponds to Kp value in yaw axis
        # after tuning and computing corresponding PID parameters, change the parameters
        
        #self.Kp = [240, 350, 13600]
        #self.Ki = [0.002 * 368, 0.002 * 380, 0.032]
        #self.Kd = [2228, 2500, 4000]

        self.Kp = [600.0 * 0.02, 300.0 * 0.02, 300.0]
        self.Ki = [200.0 * 0.002, 140.0 * 0.002, 25.0 * 0.02]
        self.Kd = [1000.0 * 0.05, 1200.0 * 0.02, 100.0] # [roll, pitch, yaw]

        # -----------------------Add other required variables for pid here ----------------------------------------------
        #
        self.prev_values = [0, 0, 0]  # [r,p,y]
        self.max_values = [1024, 1024, 1024, 1024]
        self.min_values = [0, 0, 0, 0]

        self.error = [0.0, 0.0, 0.0]  # [r,p,y]
        self.dif_error = [0.0, 0.0, 0.0]  # [r,p,y]
        self.sum_error = [0.0, 0.0, 0.0]  # Iterm [r,p,y]

        self.output = [0.0, 0.0, 0.0]  # [r,p,y]

        self.throttle = 0.0

        # Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [roll, pitch, yaw]
        #        Add variables for limiting the values like self.max_values = [1024, 1024, 1024, 1024] corresponding to [prop1, prop2, prop3, prop4]
        #                                                   self.min_values = [0, 0, 0, 0] corresponding to [prop1, prop2, prop3, prop4]
        #
        # ----------------------------------------------------------------------------------------------------------

        # # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 20  # in seconds

        # Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error
        self.pwm_pub = rospy.Publisher("/edrone/pwm", prop_speed, queue_size=1)
        # ------------------------Add other ROS Publishers here-----------------------------------------------------
        self.roll_pub = rospy.Publisher("/roll_error", Float32, queue_size=1)
        self.pitch_pub = rospy.Publisher("/pitch_error", Float32, queue_size=1)
        self.yaw_pub = rospy.Publisher("/yaw_error", Float32, queue_size=1)
        self.theta_pos_pub = rospy.Publisher("/theta_pos", Float32, queue_size=1)

        # -----------------------------------------------------------------------------------------------------------

        # Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw
        rospy.Subscriber("/drone_command", edrone_cmd, self.drone_command_callback)
        rospy.Subscriber("/edrone/imu/data", Imu, self.imu_callback)
        rospy.Subscriber("/pid_tuning_roll", PidTune, self.roll_set_pid)
        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        rospy.Subscriber("/pid_tuning_pitch", PidTune, self.pitch_set_pid)
        rospy.Subscriber("/pid_tuning_yaw", PidTune, self.yaw_set_pid)
        # ------------------------------------------------------------------------------------------------------------

        time.sleep(2.5)

        print("Initialised... OK")

        


    # Imu callback function
    # The function gets executed each time when imu publishes /edrone/imu/data

    # Note: The imu publishes various Kind of data viz angular velocity, linear acceleration, magnetometer reading (if present),
    # but here we are interested in the orientation which can be calculated by a complex algorithm called filtering which is not in the scope of this task,
    # so for your ease, we have the orientation published directly BUT in quaternion format and not in euler angles.
    # We need to convert the quaternion format to euler angles format to understand the orienataion of the edrone in an easy manner.
    # Hint: To know the message structure of sensor_msgs/Imu, execute the following command in the terminal
    # rosmsg show sensor_msgs/Imu

    def imu_callback(self, msg):

        self.drone_orientation_quaternion[0] = msg.orientation.x * 180 / math.pi

        # --------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
        self.drone_orientation_quaternion[1] = msg.orientation.y * 180 / math.pi
        self.drone_orientation_quaternion[2] = msg.orientation.z * 180 / math.pi
        self.drone_orientation_quaternion[3] = msg.orientation.w * 180 / math.pi

    def drone_command_callback(self, msg):
        self.setpoint_cmd[0] = msg.rcRoll
        self.setpoint_cmd[1] = msg.rcPitch
        self.setpoint_cmd[2] = msg.rcYaw

        #for i in range(1):
        #    self.setpoint_cmd[i] = 1500

        #print(self.setpoint_cmd[-1])



        # Throttle
        self.throttle = msg.rcThrottle

        # aux1 to enable/disable the drone
        self.aux1 = msg.aux1

        # ---------------------------------------------------------------------------------------------------------------

    # Callback function for /pid_tuning_roll
    # This function gets executed each time when /tune_pid publishes /pid_tuning_roll
    def roll_set_pid(self, roll):
        # This is just for an example. You can change the ratio/fraction value accordingly
        self.Kp[0] = roll.Kp * 0.02
        self.Ki[0] = roll.Ki * 0.002
        self.Kd[0] = roll.Kd * 0.05

    # ----------------------------Define callback function like roll_set_pid to tune pitch, yaw--------------
    # Callback function for /pid_tuning_pitch
    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 0.02
        self.Ki[1] = pitch.Ki * 0.002
        self.Kd[1] = pitch.Kd * 0.02

    # Callback function for /pid_tuning_yaw
    def yaw_set_pid(self, yaw):
        self.Kp[2] = yaw.Kp
        self.Ki[2] = yaw.Ki * 0.02
        self.Kd[2] = yaw.Kd 

    # ----------------------------------------------------------------------------------------------------------------------

    def abs_angle(self, theta):
        return theta if theta >= 0 else (2*math.pi)+theta

    def pid(self):

        if self.aux1 == 2000:
            # -----------------------------Write the PID algorithm here--------------------------------------------------------------

            # Steps:
            #   1. Convert the quaternion format of orientation to euler angles
            #   2. Convert the setpoin that is in the range of 1000 to 2000 into angles with the limit from -10 degree to 10 degree in euler angles
            #   3. Compute error in each axis. eg: error[0] = self.setpoint_euler[0] - self.drone_orientation_euler[0], where error[0] corresponds to error in roll...
            #   4. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
            #   5. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
            #   6. Use this computed output value in the equations to compute the pwm for each propeller. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
            #   7. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
            #   8. Limit the output value and the final command value between the maximum(0) and minimum(1024)range before publishing. For eg : if self.pwm_cmd.prop1 > self.max_values[1]:
            #                                                                                                                                      self.pwm_cmd.prop1 = self.max_values[1]
            #   8. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
            #   9. Add error_sum to use for integral component

            prev_yaw = self.drone_orientation_euler[-1]
            # Converting quaternion to euler angles
            (
                self.drone_orientation_euler[1],
                self.drone_orientation_euler[0],
                self.drone_orientation_euler[2],
            ) = tf.transformations.euler_from_quaternion(
                self.drone_orientation_quaternion
            )

            #print(self.drone_orientation_euler[1]*180/math.pi)

            self.theta_pos_pub.publish(self.abs_angle(self.drone_orientation_euler[-1]))

            # Convertng the range from 1000 to 2000 in the range of -10 degree to 10 degree for roll axis
            for i in range(2):
                self.setpoint_euler[i] = self.setpoint_cmd[i] * 0.02 - 30
                self.setpoint_euler[i] *= 4

            self.setpoint_euler[2] = (self.setpoint_cmd[2] - 1500 ) * math.pi / 500


            for i in range(3):
                self.drone_orientation_euler[i] = math.degrees(self.drone_orientation_euler[i])

            #print(self.drone_orientation_euler[-1])


            # Also convert the range of 1000 to 2000 to 0 to 1024 for throttle here itslef
            self.throttle = self.throttle * 1.024 - 1024

            # print(self.throttle)

            # Step 3 - Computing error in [roll, pitch, yaw]
            for i in range(3):

                #if not (self.error_check[-1] < 1 and self.error_check[-1] > -1):
                #    self.setpoint_euler[0] = 0
                #    self.setpoint_euler[1] = 0


                self.error[i] = self.setpoint_euler[i] - self.drone_orientation_euler[i]
                self.error_check[i] = self.error[i]

                if not (self.drone_orientation_euler[0] < 1 and self.drone_orientation_euler[0] > -1) and not (self.drone_orientation_euler[1] < 1 and self.drone_orientation_euler[1] > -1):
                    self.error[-1] = 0.0

                self.error[-1] = 0.0

                # print(self.error[i])

                # Step 4 - Computing the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis

                # error (for proportional)

                # change in error (for derivative)
                self.dif_error[i] = self.error[i] - self.prev_values[i]

                # sum of errors (for integral)
                self.sum_error[i] = (self.sum_error[i] + self.error[i]) * self.Ki[i]

                # Step 5 - calculating the pid output required for each axis
                self.output[i] = (
                    (self.Kp[i] * self.error[i])
                    + self.sum_error[i]
                    + (self.Kd[i] * self.dif_error[i])
                )

                # self.output[i] = self.output[i] * 1.024 - 1024

                # converting values in range -10 to 10 into values between -1024 to 1024
                # self.output[i] = self.output[i] * 102.4

                # print(self.output[i])
                """if self.output[i] > 80:
                        self.output[i] = 80
                    elif self.output[i] < -80:
                        self.output[i] = -80"""

            print format(self.error[-1], 'f')

            # Step 6 - Computing the pwm for each propeller
            # Motor mixing algorithm
            self.pwm_cmd.prop1 = (
                self.throttle - self.output[0] + self.output[1] - self.output[2]
            )  # m1 = t + r + p + y
            self.pwm_cmd.prop2 = (
                self.throttle - self.output[0] - self.output[1] + self.output[2]
            )  # m2 = t - r + p - y
            self.pwm_cmd.prop3 = (
                self.throttle + self.output[0] - self.output[1] - self.output[2]
            )  # m3 = t + r - p - y
            self.pwm_cmd.prop4 = (
                self.throttle + self.output[0] + self.output[1] + self.output[2]
            )  # m4 = t - r - p + y

            # Step 7 - Limiting the output value and the final command value
            if self.pwm_cmd.prop1 > self.max_values[0]:
                self.pwm_cmd.prop1 = self.max_values[0]

            if self.pwm_cmd.prop2 > self.max_values[1]:
                self.pwm_cmd.prop2 = self.max_values[1]

            if self.pwm_cmd.prop3 > self.max_values[2]:
                self.pwm_cmd.prop3 = self.max_values[2]

            if self.pwm_cmd.prop4 > self.max_values[3]:
                self.pwm_cmd.prop4 = self.max_values[3]

            if self.pwm_cmd.prop1 < self.min_values[0]:
                self.pwm_cmd.prop1 = self.min_values[0]

            if self.pwm_cmd.prop2 < self.min_values[1]:
                self.pwm_cmd.prop2 = self.min_values[1]

            if self.pwm_cmd.prop3 < self.min_values[2]:
                self.pwm_cmd.prop3 = self.min_values[2]

            if self.pwm_cmd.prop4 < self.min_values[3]:
                self.pwm_cmd.prop4 = self.min_values[3]

            # print(self.pwm_cmd.prop1)
            # print(self.pwm_cmd.prop2)
            # print(self.pwm_cmd.prop3)
            # print(self.pwm_cmd.prop4)

            # Step 8 - Update previous errors
            for i in range(3):
                self.prev_values[i] = self.error[i]

            # Step 9 - Add error_sum to use for integral component

            # ------------------------------------------------------------------------------------------------------------------------

        else:
            self.pwm_cmd.prop1 = 0.0
            self.pwm_cmd.prop2 = 0.0
            self.pwm_cmd.prop3 = 0.0
            self.pwm_cmd.prop4 = 0.0

        self.pwm_pub.publish(self.pwm_cmd)

        self.roll_pub.publish(self.error[0])
        self.pitch_pub.publish(self.error[1])
        self.yaw_pub.publish(self.error[2])
        #self.yaw_pub.publish(self.drone_orientation_euler[2] *180/3)

        


if __name__ == "__main__":

    e_drone = Edrone()
    r = rospy.Rate(
        e_drone.sample_time
    )  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()