import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

# Serial port configuration
SERIAL_PORT = "COM12"
BAUD_RATE = 115200
TIMEOUT = 1  # seconds

# Initialize serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)

# Data storage
sensor_data = [[] for _ in range(8)]  # Eight sensors, one list per sensor
x_data = []

# Maximum points to display on the plot
MAX_POINTS = 100

# Initialize the plot
fig, axes = plt.subplots(8, 1, figsize=(10, 12), sharex=True)
lines = []

for i, ax in enumerate(axes):
    line, = ax.plot([], [], lw=2, label=f"Sensor {i+1}")
    lines.append(line)
    ax.set_xlim(0, MAX_POINTS)
    ax.set_ylim(-10, 400)  # Adjust based on expected data range
    ax.set_ylabel(f"Sensor {i+1}")
    ax.legend(loc="upper left")

axes[-1].set_xlabel("Time Steps")

def parse_serial_data(data):
    """Parse the incoming serial data."""
    try:
        parts = data.strip().split('\t')
        if parts[0] == 'MF':  # Ensure the data starts with 'MF'
            values = list(map(int, parts[1:]))
            if len(values) == 8:
                return values
    except ValueError:
        pass
    return None

def update(frame):
    """Update the plot."""
    global x_data, sensor_data
    
    if ser.in_waiting > 0:
        raw_data = ser.readline().decode('utf-8', errors='ignore')
        parsed_values = parse_serial_data(raw_data)
        if parsed_values:
            x_data.append(len(x_data))
            
            # Append new data for each sensor
            for i in range(8):
                sensor_data[i].append(parsed_values[i])
                
                # Limit data points for each sensor
                if len(sensor_data[i]) > MAX_POINTS:
                    sensor_data[i] = sensor_data[i][-MAX_POINTS:]
            
            # Limit x_data
            if len(x_data) > MAX_POINTS:
                x_data = x_data[-MAX_POINTS:]
            
            # Update each subplot
            for i, line in enumerate(lines):
                line.set_data(x_data, sensor_data[i])
                axes[i].set_xlim(max(0, len(x_data) - MAX_POINTS), len(x_data))
    
    return lines

# Animation setup
ani = FuncAnimation(fig, update, blit=True, interval=50)

try:
    plt.tight_layout()
    plt.show()
except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
