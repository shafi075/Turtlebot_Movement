import matplotlib.pyplot as plt
import csv

# Define the path to your CSV file
csv_file_path = '/home/shafi/turtlevis_ws/error_vs_time_data.csv'

# Define a threshold for ignoring values very close to zero
threshold = 1e-5  # Adjust this value as needed

# Initialize lists to store data
time_data = []
Error_data = []

# Read the CSV file
with open(csv_file_path, 'r') as file:
    csv_reader = csv.reader(file)
    next(csv_reader)  # Skip the header row
    for row in csv_reader:
        time = float(row[0])
        Error = float(row[1])
        
        # Skip appending redundant zero values between 1 and 12 seconds
        if 1 <= time <= 12 and abs(Error) < threshold:
            continue
        
        # Append the data to the lists if it passes the filter
        time_data.append(time)
        Error_data.append(Error)

# Create the plot
plt.figure(figsize=(6.5, 5.5))

# Plot velocity vs. time
plt.plot(time_data, Error_data, color='b', label='Velocity')

# Set axis labels and title
plt.xlabel('Time (s)')
plt.ylabel('Velocity')
plt.title('Velocity vs. Time (With PID-tuning)')

grid_interval = 2

# Set x-axis limits to span the appropriate range
start_time = min(time_data)
end_time = max(time_data)
plt.xlim(start_time, end_time)

# Customize ticks on x-axis with an interval of 1
plt.xticks([i for i in range(int(start_time), int(end_time) + 3, grid_interval)])

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
