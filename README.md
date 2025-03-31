# Kalman Filter based Two-wheeled Self Balancing Mobile Base

<img src="https://github.com/user-attachments/assets/62057f54-4b3d-4a14-bb38-e1cce2c974b8" alt="tumbller" width="350"/>

https://github.com/user-attachments/assets/b0621397-8701-4904-8161-79f099c82085

# Hardware
- [ELEGOO Tumbler](https://www.elegoo.com/products/elegoo-tumbller-self-balancing-robot-car) was used for this project.
- The ELEGOO Tumbler can be programed using [Platfrom IO]([https://www.arduino.cc/en/software](https://platformio.org/)) 

# How to Send Commands
Users can send commands to control the robot’s movement, rotation, or stop function. Commands follow a predefined structured format and can be transmitted via either I2C or UART. 

**ASCII Format**:  
The commands sent in ASCII are formatted as a comma-separated string:
```
<command>,<command_value>,<command_speed>
```
The available commands are listed in are explained in table below.
| **Command**   | **Value** | **Description**                                           |
|---------------|-----------|-----------------------------------------------------------|
| `Stop`        | 0         | Stops the robot's movement.                               |
| `Move`        | 1         | Moves forward or backward (in cm) at a given speed.      |
| `Rotate`      | 2         | Rotates the robot by an angle (in degrees) at a given speed. |
| `INVALID`     | 3         | Represents an invalid or unrecognized command.            |

**Example Command:**
- Stop the robot: `0,0,0`
- Move forward 100 cm at 50% speed: `1,100,50`
- Rotate 90° at 30% speed: `2,90,30`

# Bluetooth Control
User can also connect to the robot through "DX-BT16" Bluetooth module. User can install andriod applications like [Serial Bluetooth Terminal](https://play.google.com/store/apps/details?id=de.kai_morich.serial_bluetooth_terminal&pcampaignid=web_share) and send UART commands wirelessly to the robot.

# Pin Config
<img src= "https://github.com/haider-rizvi-github/Self-Balancing-Robot/blob/main/reference%20material/Pin%20ports.png"  width="520">

# Video Demo
https://github.com/user-attachments/assets/72d14e31-a2a4-4ef7-abc6-7005c3e53e92
