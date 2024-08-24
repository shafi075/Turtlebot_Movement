#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion
import csv

class GoalSubscriber:
    def __init__(self):
        rospy.init_node('goal_subscriber', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/target_coordinates', Float32MultiArray, self.send_goal)
        rospy.Subscriber('/odom', Odometry, self.update_position)
        self.current_position = (0, 0)
        self.current_orientation = 0
        self.rate = rospy.Rate(10)

        # Fixed speeds for linear and angular velocities
        self.fixed_linear_speed = 0.7  # Fixed linear speed
        self.fixed_angular_speed = 0.8  # Fixed angular speed

        # Open CSV file for logging path
        self.file = open('/home/shafi/turtlevis_ws/path_without_pid.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.file)
        self.csv_writer.writerow(['Time', 'X Position', 'Y Position'])

        self.start_time = rospy.get_time()

    def update_position(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.current_orientation = yaw

        # Write the position to the CSV file
        current_time = rospy.get_time() - self.start_time
        self.csv_writer.writerow([current_time, self.current_position[0], self.current_position[1]])

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

            # Use fixed angular speed
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = self.fixed_angular_speed if angle_diff > 0 else -self.fixed_angular_speed
            self.pub.publish(twist)
            self.rate.sleep()

            # Wait until the orientation is updated
            rospy.sleep(0.1)

        # Phase 2: Move towards the goal
        while distance > 0.05 and not rospy.is_shutdown():
            distance = math.sqrt((target_x - self.current_position[0]) ** 2 +
                                 (target_y - self.current_position[1]) ** 2)
            angle_to_goal = math.atan2(target_y - self.current_position[1],
                                       target_x - self.current_position[0])
            angle_diff = angle_to_goal - self.current_orientation
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

            # Use fixed linear and angular speeds
            twist = Twist()
            twist.linear.x = self.fixed_linear_speed if distance > 0 else 0
            twist.angular.z = self.fixed_angular_speed if angle_diff > 0 else -self.fixed_angular_speed
            self.pub.publish(twist)
            self.rate.sleep()

        self.stop_robot()

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.pub.publish(twist)
        self.file.close()  # Close the CSV file when done

if __name__ == '__main__':
    try:
        subscriber = GoalSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
