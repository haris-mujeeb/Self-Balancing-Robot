import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Serial port configuration
SERIAL_PORT = "COM12"
BAUD_RATE = 115200
TIMEOUT = 1  # seconds

# Initialize serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)

# Circular frame setup
NUM_SENSORS = 8
FRAME_RADIUS = 80  # mm
angles = np.linspace(0, 2 * np.pi, NUM_SENSORS, endpoint=False)  # Equally spaced angles
sensor_distances = [FRAME_RADIUS] * NUM_SENSORS  # Initialize distances to frame radius

# Initialize the plot
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
points, = ax.plot(angles, sensor_distances, 'o-', label="ToF Sensors")
ax.set_ylim(0, 500)  # Adjust to the maximum expected distance

# Labels for sensors
for angle, label in zip(angles, range(1, NUM_SENSORS + 1)):
    ax.text(angle, FRAME_RADIUS + 20, f"S{label}", ha='center', va='center')

ax.legend(loc='upper right')

def parse_serial_data(data):
    """Parse the incoming serial data."""
    try:
        parts = data.strip().split('\t')
        if parts[0] == 'MF':  # Ensure the data starts with 'MF'
            values = list(map(int, parts[1:]))
            if len(values) == NUM_SENSORS:
                return values
    except ValueError:
        pass
    return None

def update(frame):
    """Update the animation."""
    global sensor_distances
    
    if ser.in_waiting > 0:
        raw_data = ser.readline().decode('utf-8', errors='ignore')
        parsed_values = parse_serial_data(raw_data)
        if parsed_values:
            sensor_distances = parsed_values  # Update distances
            points.set_ydata(sensor_distances)  # Update plot data
    
    return points,

# Animation setup
ani = FuncAnimation(fig, update, blit=True, interval=100)

try:
    plt.show()
except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
