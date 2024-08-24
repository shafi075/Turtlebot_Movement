#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion
import csv

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0

    def compute(self, target, current):
        self.error = target - current
        self.integral += self.error
        self.derivative = self.error - self.previous_error
        self.previous_error = self.error
        return self.Kp * self.error + self.Ki * self.integral + self.Kd * self.derivative

class GoalSubscriber:
    def __init__(self):
        rospy.init_node('goal_subscriber', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/target_coordinates', Float32MultiArray, self.send_goal)
        rospy.Subscriber('/odom', Odometry, self.update_position)
        self.current_position = (0, 0)
        self.current_orientation = 0
        self.rate = rospy.Rate(10)

        # PID-tuning for linear and angular velocities
        self.linear_pid = PID(Kp=1, Ki=0, Kd=0.1)
        self.angular_pid = PID(Kp=1, Ki=0, Kd=0.1)

        # Maximum linear and angular speeds
        self.max_linear_speed = 0.5   # adjustable Linear speed
        self.max_angular_speed = 1.0  # adjustable angular speed

        # Open CSV files for logging
        self.file_trajectory = open('/home/shafi/turtlevis_ws/trajectory_data.csv', 'w', newline='')
        self.csv_writer_trajectory = csv.writer(self.file_trajectory)
        self.csv_writer_trajectory.writerow(['Time', 'X Position', 'Y Position'])

        self.file_response = open('/home/shafi/turtlevis_ws/response_curve_data.csv', 'w', newline='')
        self.csv_writer_response = csv.writer(self.file_response)
        self.csv_writer_response.writerow(['Time', 'Linear Speed'])

        self.file_error = open('/home/shafi/turtlevis_ws/error_vs_time_data.csv', 'w', newline='')
        self.csv_writer_error = csv.writer(self.file_error)
        self.csv_writer_error.writerow(['Time', 'Error'])

        self.start_time = rospy.get_time()

    def update_position(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.current_orientation = yaw

        # Write the position to the CSV file
        current_time = rospy.get_time() - self.start_time
        self.csv_writer_trajectory.writerow([current_time, self.current_position[0], self.current_position[1]])

    def send_goal(self, data):
        target_x, target_y = data.data
        rospy.loginfo(f"Received goal: x={target_x}, y={target_y}")

        distance = math.sqrt((target_x - self.current_position[0]) ** 2 +
                             (target_y - self.current_position[1]) ** 2)
        angle_to_goal = math.atan2(target_y - self.current_position[1],
                                   target_x - self.current_position[0])

        # Phase 1: Rotate towards the goal
        while abs(angle_to_goal - self.current_orientation) > 0.05 and not rospy.is_shutdown():
            angle_diff = angle_to_goal - self.current_orientation
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

            angular_speed = self.angular_pid.compute(angle_diff, 0)
            angular_speed = max(min(angular_speed, self.max_angular_speed), -self.max_angular_speed)

            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = angular_speed
            self.pub.publish(twist)
            self.rate.sleep()

            # Write the error and speed to the CSV files
            current_time = rospy.get_time() - self.start_time
            self.csv_writer_error.writerow([current_time, distance])
            self.csv_writer_response.writerow([current_time, 0])

            # Wait until the orientation is updated
            rospy.sleep(0.1)

        # Phase 2: Move towards the goal
        while distance > 0.01 and not rospy.is_shutdown():
            distance = math.sqrt((target_x - self.current_position[0]) ** 2 +
                                 (target_y - self.current_position[1]) ** 2)
            angle_to_goal = math.atan2(target_y - self.current_position[1],
                                       target_x - self.current_position[0])
            angle_diff = angle_to_goal - self.current_orientation
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

            linear_speed = self.linear_pid.compute(distance, 0)
            angular_speed = self.angular_pid.compute(angle_diff, 0)

            # Apply speed limits
            linear_speed = max(min(linear_speed, self.max_linear_speed), -self.max_linear_speed)
            angular_speed = max(min(angular_speed, self.max_angular_speed), -self.max_angular_speed)

            twist = Twist()
            twist.linear.x = linear_speed
            twist.angular.z = angular_speed
            self.pub.publish(twist)
            self.rate.sleep()

            # Write the error and speed to the CSV files
            current_time = rospy.get_time() - self.start_time
            self.csv_writer_error.writerow([current_time, distance])
            self.csv_writer_response.writerow([current_time, linear_speed])

        self.stop_robot()

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.pub.publish(twist)
        self.file_trajectory.close()
        self.file_response.close()
        self.file_error.close()  # Close the CSV files when done

if __name__ == '__main__':
    try:
        subscriber = GoalSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
