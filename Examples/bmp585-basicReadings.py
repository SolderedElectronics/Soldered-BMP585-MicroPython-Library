# FILE: bmp585-basicReadings.py
# AUTHOR: Soldered Electronics
# BRIEF: Reads pressure and temperature from the BMP585 sensor over I2C
# WORKS WITH: BMP585 Barometric Pressure Sensor breakout: www.solde.red/333189
# LAST UPDATED: 2026-09-18

from bmp585 import BMP585, BMP5_OK
from machine import I2C, Pin
import time

# Change these to match how your board wires I2C
i2c = I2C(0, scl=Pin(9), sda=Pin(8))
sensor = BMP585(i2c)

if sensor.status != BMP5_OK:
    raise Exception("Failed to initialize BMP585: " + sensor.status_string())

print("BMP585 connected!")

while True:
    if sensor.get_sensor_data() == BMP5_OK:
        print(
            "Pressure: {:.2f} Pa\tTemperature: {:.2f} degC".format(
                sensor.data.pressure, sensor.data.temperature
            )
        )
    else:
        print("Failed to read sensor data:", sensor.status_string())

    time.sleep_ms(100)
