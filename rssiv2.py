from bluepy.btle import Scanner
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import sh1106
from time import sleep
import smbus
from imusensor.MPU9250 import MPU9250
import math

# Constants (adjust as needed)
RSSI_THRESHOLD = -70
ACCEL_MOVE_THRESHOLD = 0.5  # Example threshold for movement

# Initialization
serial = i2c(port=2, address=0x3C)
device = sh1106(serial)
address = 0x68
bus = smbus.SMBus(1)
imu = MPU9250.MPU9250(bus, address)
imu.begin()

# Magnetometer calibration (if needed, add calibration routine here)

def calculate_direction(mag_x, mag_y, mag_z):
    heading = math.atan2(mag_y, mag_x) * 180 / math.pi

    # Adjust for declination (if you know the magnetic declination for your location)
    # Example: If declination is 10 degrees East, uncomment the following line
    # heading += 10 

    # Convert to cardinal directions
    if heading < 0:
        heading += 360

    if 0 <= heading < 22.5:
        return "N"
    elif 22.5 <= heading < 67.5:
        return "NE"
    elif 67.5 <= heading < 112.5:
        return "E"
    elif 112.5 <= heading < 157.5:
        return "SE"
    elif 157.5 <= heading < 202.5:
        return "S"
    elif 202.5 <= heading < 247.5:
        return "SW"
    elif 247.5 <= heading < 292.5:
        return "W"
    elif 292.5 <= heading < 337.5:
        return "NW"
    else:
        return "N"

while True:
    try:
        # RSSI measurement
        ble_list = Scanner().scan(10.0)
        max_rssi = -100
        for dev in ble_list:
            max_rssi = max(max_rssi, dev.rssi)

        # IMU measurement
        imu.readSensor()
        imu.computeOrientation()

        accel_x, accel_y, accel_z = imu.AccelVals
        mag_x, mag_y, mag_z = imu.MagVals

        # Movement detection
        movement_detected = False
        if abs(accel_x) > ACCEL_MOVE_THRESHOLD or abs(accel_y) > ACCEL_MOVE_THRESHOLD  or abs(accel_z) > ACCEL_MOVE_THRESHOLD:
            movement_detected = True

        # Direction
        direction_str = calculate_direction(mag_x, mag_y, mag_z)  

        # Output
        print("RSSI:", max_rssi)
        print("Movement:", movement_detected)
        print("Direction:", direction_str)

        # OLED display (update accordingly)
        with canvas(device) as draw:
            # ... Update the OLED display with RSSI, movement, direction

    except Exception as e:
        print(f"Error occurred: {str(e)}")

    sleep(1)  # Adjust polling interval as needed 