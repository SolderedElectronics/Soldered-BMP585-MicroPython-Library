# FILE: bmp585-forcedModeCustomConfig.py
# AUTHOR: Soldered Electronics
# BRIEF: Configures oversampling/IIR filtering and takes single-shot readings
#        in forced power mode with the BMP585 sensor over I2C. Forced mode
#        triggers one measurement then returns the sensor to standby
#        automatically, useful for low-power applications where you only
#        need occasional readings.
# WORKS WITH: BMP585 Barometric Pressure Sensor breakout: www.solde.red/333189
# LAST UPDATED: 2026-09-18

from bmp585 import (
    BMP585,
    BMP5_OK,
    BMP5_OVERSAMPLING_1X,
    BMP5_ODR_50_HZ,
    BMP5_IIR_FILTER_BYPASS,
    BMP5_POWERMODE_FORCED,
    BMP5_INT_ASSERTED_DRDY,
)
from machine import I2C, Pin
import time

# Change these to match how your board wires I2C
i2c = I2C(0, scl=Pin(9), sda=Pin(8))
sensor = BMP585(i2c)

if sensor.status != BMP5_OK:
    raise Exception("Failed to initialize BMP585: " + sensor.status_string())

print("BMP585 connected!")

# Lower the oversampling from the driver's default since forced mode is
# typically used for infrequent, quick readings. ODR is ignored in forced
# mode, but still needs to be set to a valid value
sensor.set_osr_odr_config(BMP5_OVERSAMPLING_1X, BMP5_OVERSAMPLING_1X, BMP5_ODR_50_HZ)

# Bypass the IIR filter since forced mode readings aren't continuous enough
# for the filter to settle
sensor.set_iir_config(BMP5_IIR_FILTER_BYPASS, BMP5_IIR_FILTER_BYPASS)

# Trigger the first measurement
sensor.set_mode(BMP5_POWERMODE_FORCED)

while True:
    # Wait for the measurement to be ready
    interrupt_status = sensor.get_interrupt_status()

    if interrupt_status is not None and interrupt_status & BMP5_INT_ASSERTED_DRDY:
        if sensor.get_sensor_data() == BMP5_OK:
            print(
                "Pressure: {:.2f} Pa\tTemperature: {:.2f} degC".format(
                    sensor.data.pressure, sensor.data.temperature
                )
            )
        else:
            print("Failed to read sensor data:", sensor.status_string())

        # Trigger the next measurement
        sensor.set_mode(BMP5_POWERMODE_FORCED)

        # Only take a reading once per second
        time.sleep_ms(1000)
