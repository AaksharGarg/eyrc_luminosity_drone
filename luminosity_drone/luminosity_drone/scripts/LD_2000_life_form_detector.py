#!/usr/bin/env python3

"""

This python file runs a ROS-node of name drone_control which holds the position of Swift-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
								

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
"""

# Importing the required libraries

from swift_msgs.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
from led_detection import LEDDetector
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from luminosity_drone.msg import Biolocation

class swift:
    """docstring for swift"""

    def __init__(self):
        rospy.init_node(
            "drone_control"
        )  # initializing ros node with name drone_control
        self.travel_flag = True
        self.bridge = CvBridge()  # Creating an Instance of CV Bridge
        self.image_sub = rospy.Subscriber(
            "/swift/camera_rgb/image_raw", Image, self.image_callback
        )  # Subsciber for the Image feed
        # This corresponds to your current position of drone. This value must be updated each time in your whycon callback
        # [x,y,z]
        self.drone_position = [0.0, 0.0, 0.0]
        self.drone_camera=[0,0]
        # [x_setpoint, y_setpoint, z_setpoint]
        self.point_num = -1
        self.setpoint = [
            [0, 0, 15],
            [-5, -5, 15],
            [0, -5, 15],
            [5, -5, 15],
            [5, 0, 15],
            [0, 0, 15],
            [-5, 0, 15],
            [-5, 5, 15],
            [0, 5, 15],
            [5, 5, 15],
        ]
        # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly

        # Declaring a cmd of message type swift_msgs and initializing values
        self.cmd = swift_msgs()
        self.org = Biolocation()
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX1 = 0
        self.cmd.rcAUX2 = 0
        self.cmd.rcAUX3 = 0
        self.cmd.rcAUX4 = 0

        # initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
        # after tuning and computing corresponding PID parameters, change the parameters

        self.Kp = [10, 20, 75]  # 8 15 70
        self.Ki = [0.008, 2, 0.25]  # 0.008 1 2
        self.Kd = [6.7, 6.7, 21]  # 1.2 3200 34

        # -----------------------Add other required variables for pid here ----------------------------------------------
        self.error = [0.0, 0.0, 0.0]  # [x,y,z]
        self.prev_error = [0.0, 0.0, 0.0]  # [roll , pitch , throttle]
        self.sum_error = [0.0, 0.0, 0.0]  # [roll , pitch , throttle]

        self.prev_time = rospy.get_time()

        # Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]
        # #		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
        # 													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
        # 																	You can change the upper limit and lower limit accordingly.
        # ----------------------------------------------------------------------------------------------------------

        # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        # self.sample_time = 0.033 # in seconds

        # Publishing /drone_command, /alt_error, /pitch_error, /roll_error
        self.command_pub = rospy.Publisher("/drone_command", swift_msgs, queue_size=1)
        self.organism_pub = rospy.Publisher("/astrobiolocation",Biolocation , queue_size=1)
        # ------------------------Add other ROS Publishers here-----------------------------------------------------
        self.alt_error_pub = rospy.Publisher("/alt_error", Float64, queue_size=1)
        self.pitch_error_pub = rospy.Publisher("/pitch_error", Float64, queue_size=1)
        self.roll_error_pub = rospy.Publisher("/roll_error", Float64, queue_size=1)

        # -----------------------------------------------------------------------------------------------------------

        # Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
        rospy.Subscriber("whycon/poses", PoseArray, self.whycon_callback)
        rospy.Subscriber("/pid_tuning_altitude", PidTune, self.altitude_set_pid)
        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        rospy.Subscriber("pid_tuning_roll", PidTune, self.roll_set_pid)
        rospy.Subscriber("/pid_tuning_pitch", PidTune, self.pitch_set_pid)

        # ------------------------------------------------------------------------------------------------------------
        self.arm()  # ARMING THE DRONE

    def image_callback(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(
            data, "bgr8"
        )  # Converting Image to CV2 comatible datatype
        try:
            led_detector = LEDDetector(self.cv_image)
            # print("\n", len(led_detector.contour_list), "-", led_detector.contour_list)
            for i in range(len(led_detector.contour_list)):
                    # print(i,led_detector.contour_list[i)
                    self.drone_camera[0]+=led_detector.contour_list[i][0]
                    self.drone_camera[1]+=led_detector.contour_list[i][1]
                    # self.drone_camera[2]+=led_detector.contour_list[i]
            self.drone_camera[0]=self.drone_camera[0]/len(led_detector.contour_list)
            self.drone_camera[1]=self.drone_camera[1]/len(led_detector.contour_list)
            
            # print(self.drone_camera)
            if len(led_detector.contour_list) > 1:

                self.travel_flag = False
                self.setpoint = [self.setpoint[self.point_num]]
                self.point_num = 0
                if len(led_detector.contour_list)==2:
                    self.org.organism_type="alien_a"
                elif len(led_detector.contour_list)==3:
                    self.org.organism_type="alien_b"
                elif len(led_detector.contour_list)==4:
                    self.org.organism_type="alien_c"
                self.org.whycon_x =(self.drone_camera[0]*(self.drone_position[0]/250))
                self.org.whycon_y =(self.drone_camera[1]*(self.drone_position[1]/250))
                self.org.whycon_z=self.drone_position[2]
                self.organism_pub.publish(self.org)
                # print(self.drone_camera[0]*(self.drone_position[0]/250))
                # self.setpoint = [(self.drone_camera[0]*(self.drone_position[0]/250)),(self.drone_camera[1]*(self.drone_position[1]/250)),15]

                self.point_num = 0
        except:
            pass
        cv2.waitKey(1)

    # Disarming condition of the drone
    def disarm(self):
        self.cmd.rcAUX1 = 0
        self.cmd.rcAUX2 = 0
        self.cmd.rcAUX3 = 0
        self.cmd.rcAUX4 = 0
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)

    # Arming condition of the drone : Best practise is to disarm and then arm the drone.
    def arm(self):
        self.disarm()

        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1590
        self.cmd.rcAUX1 = 0
        self.cmd.rcAUX2 = 0
        self.cmd.rcAUX3 = 0
        self.cmd.rcAUX4 = 0

        self.command_pub.publish(self.cmd)  # Publishing /drone_command
        rospy.sleep(1)

    # Whycon callback function
    # The function gets executed each time when /whycon node publishes /whycon/poses
    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x

        # --------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z

        # ---------------------------------------------------------------------------------------------------------------

    # Callback function for /pid_tuning_altitude
    # This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
    def altitude_set_pid(self, alt):
        self.Kp[2] = (
            alt.Kp * 0.6
        )  # #66 This is just for an example. You can change the ratio/fraction value accordingly
        self.Ki[2] = alt.Ki * 0.0008  # 2
        self.Kd[2] = alt.Kd * 0.3  # 2140

    # ----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------
    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.Kp * 0.6  # 0.6
        self.Ki[1] = pitch.Ki * 0.0008  # 0.0008
        self.Kd[1] = pitch.Kd * 0.3  # 0.3

    def roll_set_pid(self, roll):
        self.Kp[0] = roll.Kp * 0.6
        self.Ki[0] = roll.Ki * 0.0008
        self.Kd[0] = roll.Kd * 0.3

    # ----------------------------------------------------------------------------------------------------------------------

    def pid(self):
        # -----------------------------Write the PID algorithm here--------------------------------------------------------------
        if self.travel_flag:
            print("Continuing")
            if (
                abs(self.error[0]) <= 0.2
                and abs(self.error[1]) <= 0.2
                and abs(self.error[2]) <= 0.2
            ):
                if len(self.setpoint) > (self.point_num + 1):
                    self.point_num += 1

                    print(self.drone_position, "################", self.point_num)

            self.error[0] = -(self.drone_position[0] - self.setpoint[self.point_num][0])
            self.error[1] = self.drone_position[1] - self.setpoint[self.point_num][1]
            self.error[2] = self.drone_position[2] - self.setpoint[self.point_num][2]
        else:
            self.error[0] = -(self.drone_position[0] - self.setpoint[self.point_num][0])
            self.error[1] = self.drone_position[1] - self.setpoint[self.point_num][1]
            self.error[2] = self.drone_position[2] - self.setpoint[self.point_num][2]

        current_time = rospy.get_time()
        dt = current_time - self.prev_time

        self.prev_time = current_time
        # .................FOR THROTTLE.................

        # Calculate PID terms
        # Proportional term
        Pt = self.error[2] * self.Kp[2]

        # Integral term (accumulated error)
        It = self.sum_error[2] * self.Ki[2]
        self.sum_error[2] += self.error[2]

        # Derivative term (rate of change of error)
        Dt = (self.error[2] - self.prev_error[2]) / dt * self.Kd[2]

        # Calculate the throttle command
        self.cmd.rcThrottle = int(1500 + Pt + It + Dt)

        # Limit the throttle command within the specified range
        if self.cmd.rcThrottle > 2000:
            self.cmd.rcThrottle = 2000
        elif self.cmd.rcThrottle < 1000:
            self.cmd.rcThrottle = 1000

            # Update previous error for the z-axis
        self.prev_error[2] = self.error[2]

        # ....................FOR ROLL...................

        # Calculate PID terms
        # Proportional term
        Pr = self.error[0] * self.Kp[0]

        # Integral term (accumulated error)
        Ir = self.sum_error[0] * self.Ki[0]
        self.sum_error[0] += self.error[0]

        # Derivative term (rate of change of error)
        Dr = (self.error[0] - self.prev_error[0]) / dt * self.Kd[0]

        # Calculate the throttle command
        self.cmd.rcRoll = int(1500 + Pr + Ir + Dr)

        # Limit the throttle command within the specified range
        if self.cmd.rcRoll > 2000:
            self.cmd.rcRoll = 2000
        elif self.cmd.rcRoll < 1000:
            self.cmd.rcRoll = 1000

            # Update previous error for the z-axis
        self.prev_error[0] = self.error[0]

        # ....................FOR PITCH...................

        # Calculate PID terms
        # Proportional term
        Pp = self.error[1] * self.Kp[1]

        # Integral term (accumulated error)
        Ip = self.sum_error[1] / 100 * self.Ki[1]
        self.sum_error[1] += self.error[1]

        # Derivative term (rate of change of error)
        Dp = (self.error[1] - self.prev_error[1]) / dt * self.Kd[1]

        # Calculate the throttle command
        self.cmd.rcPitch = int(1500 + Pp + Ip + Dp)

        # Limit the throttle command within the specified range
        if self.cmd.rcPitch > 2000:
            self.cmd.rcPitch = 2000
        elif self.cmd.rcPitch < 1000:
            self.cmd.rcPitch = 1000

            # Update previous error for the z-axis
        self.prev_error[1] = self.error[1]

        # ------------------------------------------------------------------------------------------------------------------------
        self.command_pub.publish(self.cmd)


if __name__ == "__main__":
    swift_drone = swift()
    r = rospy.Rate(
        30
    )  # specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
    while not rospy.is_shutdown():
        swift_drone.pid()
        r.sleep()
