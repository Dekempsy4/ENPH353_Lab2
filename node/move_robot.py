#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
import numpy as np


rospy.init_node('move_robots')

class image_analyzer: 
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw", Image, self.callback) 
        self.previous_index = 0
        self.prev_error = 0
        self.frame_height_from_bottom = 5


    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        self.analyze_image(cv_image)

    # used proportion and derivative response to adjust yaw
    def pid_control(self, setpoint, current_value):
        Kp = 0.020
        Kd = 0.0013
        error = setpoint - current_value
        derivative = (error - self.prev_error)
        output = (Kp * error) + (Kd * derivative)
        self.prev_error = error
        print(output)
        return output

    def analyze_image(self, cv_image):
        frame_height, frame_width = cv_image.shape[:2]  # [:2] extracts height and width

        # convert frame to grayscale more effectively compare color differences between pixels
        gray_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        analyzed_row = gray_frame[frame_height - self.frame_height_from_bottom]

        mean_value = np.mean(analyzed_row)

        middle_list = []
        for i in range(analyzed_row.size):
            # introducing a threshold properly identifies frames with no road
            if analyzed_row[i] < mean_value - 10:
                middle_list.append(i)

        # Convert the list to a NumPy array
        middle_array: np.ndarray = np.array(middle_list)

        # If no road is detected, the ball stays in it's last position
        if middle_array.size == 0:
            ball_center_index = self.previous_index
            # print("road lost!")
        else:
            ball_center_index = np.mean(middle_array)
            self.previous_index = ball_center_index

        center_pt = (int(ball_center_index), frame_height - self.frame_height_from_bottom) #(x, y)
        radius = 10
        color = (255, 0, 255)
        line_thickness = -1
        cv2.circle(cv_image, center_pt, radius, color, line_thickness)

        yaw = self.pid_control((frame_width / 2), ball_center_index)
        drive_robot(yaw)

        # opens a new window with the analyzed images
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)


pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(60)
move = Twist()
ic = image_analyzer()


# send movement command to the robots
def drive_robot(yaw):
    move.linear.x = 1.0
    move.angular.z = yaw


while not rospy.is_shutdown():
   pub.publish(move)
   rate.sleep()
