import matplotlib.pyplot as plt
import csv

# Define the path to your CSV file
csv_file_path = '/home/shafi/turtlevis_ws/path_without_pid.csv'

# Define a threshold for ignoring values very close to zero
threshold = 1e-4 # Adjust this value as needed

# Initialize lists to store data
time_data = []
x_data = []
y_data = []

# Read the CSV file
with open(csv_file_path, 'r') as file:
    csv_reader = csv.reader(file)
    next(csv_reader)  # Skip the header row
    for row in csv_reader:
        time = float(row[0])
        x = float(row[1])
        y = float(row[2])
        
        # Filter out values very close to zero
        if abs(x) > threshold and abs(y) > threshold:
            time_data.append(time)
            x_data.append(x)
            y_data.append(y)

# Create the plot
plt.figure(figsize=(6.5, 5.5))

# Plot x position vs. time
plt.plot(time_data, x_data, color='b', label='x-Position')

# Plot y position vs. time
plt.plot(time_data, y_data, color='g', label='y-Position')

# Set axis labels and title
plt.xlabel('Time (s)')
plt.ylabel('Position')
plt.title('Position vs. Time (without PID-tuning)')

# Define grid interval
grid_interval = 1  # Grid lines every 1 unit of time

# Set x-axis limits to span an appropriate range that is divisible by the grid interval
start_time = min(time_data)
end_time = max(time_data)
plt.xlim(start_time, end_time)  # Ensure there's space around the data
plt.xticks(range(int(start_time)+1, int(end_time)+3))  # Set x-ticks

# Customize grid
plt.grid(True, which='both', linestyle='--', linewidth=0.5, color='black')
plt.minorticks_on()
plt.grid(True, which='minor', linestyle=':', linewidth=0.5, color='lightgray')

# Add legend
plt.legend()

# Adjust layout
plt.tight_layout()

# Show the plot
plt.show()
