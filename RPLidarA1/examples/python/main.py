import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.lines import Line2D
import numpy as np

# Serial port configuration
serialPort = '/dev/tty.wchusbserial1412330'  # Modify according to actual situation
baudRate = 115200
timeout = 2

# Try to initialize the serial port connection
try:
    ser = serial.Serial(serialPort, baudRate, timeout=timeout)
    print(f"Serial port {serialPort} is open, baud rate {baudRate}")
except serial.SerialException as e:
    print(f"Failed to open serial port {serialPort}: {e}")
    exit()

# Initialize a dictionary to store angles and corresponding distance values
angle_distance_cache = {}

def update_radar(frame):
    global angle_distance_cache
    while ser.in_waiting:
        data = ser.readline().decode('utf-8').strip()
        try:
            angle_str, distance_str = data.split(' ')
            # Adjust the angle for 180 degrees resolution, grouping every 2 degrees
            angle = round((float(angle_str.split(':')[1]) % 360) / 2) * 2
            distance = float(distance_str.split(':')[1])
            # Update or add the angle and distance value to the cache
            angle_distance_cache[angle] = distance
        except ValueError:
            print(f"Data parsing error: {data}")

    ax.clear()
    ax.set_theta_zero_location('N')
    # Set to counterclockwise direction
    ax.set_theta_direction(-1)
    # Set the maximum display radius
    max_display_radius = 5000  # This value can be adjusted as needed
    ax.set_ylim(0, max_display_radius)
    
    # Skip drawing if the dictionary is empty
    if not angle_distance_cache:
        return
    
    # Prepare the data for drawing
    angles = np.deg2rad(list(angle_distance_cache.keys()))
    distances = list(angle_distance_cache.values())
    
    # Draw all cached data points, setting the size of the points to 5 (this can be adjusted as needed)
    ax.scatter(angles, distances, s=5)

# Set up the animation
fig = plt.figure()
ax = fig.add_subplot(111, polar=True)
ani = animation.FuncAnimation(fig, update_radar, interval=1000)

plt.show()

ser.close()
print(f"Serial port {serialPort} is closed")
