#!/usr/bin/env python3

import tkinter as tk
import rospy
from std_msgs.msg import Float32MultiArray

class myController:
    def __init__(self):

        # Initialize ROS node and publisher
        rospy.init_node('gui_controller', anonymous=True)
        self.pub = rospy.Publisher('/target_coordinates', Float32MultiArray, queue_size=10)
        rospy.loginfo("GUI Controller node initialized")

        # Initialize Tkinter window
        self.root = tk.Tk()
        self.root.geometry("700x400") 
        self.root.title("Bot Controller")
        self.root.configure(bg="#2c3e50")

        # Create a frame for the input fields
        self.frame = tk.Frame(self.root, bg="#34495e", padx=30, pady=30, relief=tk.RAISED)
        self.frame.place(relx=0.5, rely=0.3, anchor=tk.CENTER)

        # Add a title label
        self.title_label = tk.Label(self.frame, text="Set Target Coordinates", font=("Arial", 18, "bold"), bg="#34495e", fg="#ecf0f1")
        self.title_label.grid(row=0, column=0, columnspan=2, pady=15)

        # Add x coordinate input
        self.x_label = tk.Label(self.frame, text="x:", font=("Arial", 16), bg="#34495e", fg="#ecf0f1")
        self.x_label.grid(row=1, column=0, padx=10, pady=10, sticky="e")

        self.x_entry = tk.Entry(self.frame, font=("Arial", 16), width=15)
        self.x_entry.grid(row=1, column=1, padx=10, pady=10)

        # Add y coordinate input
        self.y_label = tk.Label(self.frame, text="y:", font=("Arial", 16), bg="#34495e", fg="#ecf0f1")
        self.y_label.grid(row=2, column=0, padx=10, pady=10, sticky="e")

        self.y_entry = tk.Entry(self.frame, font=("Arial", 16), width=15)
        self.y_entry.grid(row=2, column=1, padx=10, pady=10)

        # Add status label with color feedback
        self.status_label = tk.Label(self.root, text="", font=("Arial", 14), bg="#2c3e50", fg="#1abc9c")
        self.status_label.place(relx=0.5, rely=0.6, anchor=tk.CENTER)

        # Add Set Coordinates button with brighter green background
        self.set_button = tk.Button(self.root, text="Set Coordinates", font=("Arial", 16), bg="#32CD32", fg="#ffffff", command=self.set_coordinates) 
        self.set_button.place(relx=0.3, rely=0.8, anchor=tk.CENTER)

        # Add Reset button with bright red background
        self.reset_button = tk.Button(self.root, text="Reset", font=("Arial", 16), bg="#ff4c4c", fg="#ffffff", command=self.reset_fields)
        self.reset_button.place(relx=0.5, rely=0.8, anchor=tk.CENTER)

        # Add Exit button
        self.exit_button = tk.Button(self.root, text="Exit", font=("Arial", 16), bg="#95a5a6", fg="#ffffff", command=self.root.quit)
        self.exit_button.place(relx=0.7, rely=0.8, anchor=tk.CENTER)

        self.root.mainloop()

    def set_coordinates(self):
        try:
            x_value = float(self.x_entry.get())
            y_value = float(self.y_entry.get())
            
            coordinates = Float32MultiArray()
            coordinates.data = [x_value, y_value]
            
            # Publish the target coordinates
            self.pub.publish(coordinates)
            rospy.loginfo(f"Published goal: x={x_value}, y={y_value}")

            # Update the status label with success and green color
            self.status_label.config(text=f"Coordinates set to x: {x_value}, y: {y_value}", fg="#1abc9c")
        except ValueError:
            rospy.logerr("Invalid input. Please enter numerical values.")
            self.status_label.config(text="Invalid input. Please enter numerical values.", fg="#e74c3c")

    def reset_fields(self):
        # Clear the entry fields and status label
        self.x_entry.delete(0, tk.END)
        self.y_entry.delete(0, tk.END)
        self.status_label.config(text="", fg="#1abc9c")

if __name__ == '__main__':
    try:
        myController()
    except rospy.ROSInterruptException:
        pass
