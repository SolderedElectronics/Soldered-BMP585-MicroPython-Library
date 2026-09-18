# FILE: bmp585-dataReadyInterrupt.py
# AUTHOR: Soldered Electronics
# BRIEF: Uses the BMP585's physical interrupt pin to know when a new
#        pressure/temperature reading is ready, instead of polling the
#        interrupt status register
# WORKS WITH: BMP585 Barometric Pressure Sensor breakout: www.solde.red/333189
# LAST UPDATED: 2026-09-18

from bmp585 import (
    BMP585,
    BMP5_OK,
    BMP5_PULSED,
    BMP5_ACTIVE_HIGH,
    BMP5_INTR_PUSH_PULL,
    BMP5_INT_ASSERTED_DRDY,
)
from machine import I2C, Pin

# Hardware setup: connect the breakout board's INT pin to INT_PIN below
INT_PIN = 5

data_ready = False


def int_callback(pin):
    global data_ready
    data_ready = True


# Change these to match how your board wires I2C
i2c = I2C(0, scl=Pin(9), sda=Pin(8))
sensor = BMP585(i2c)

if sensor.status != BMP5_OK:
    raise Exception("Failed to initialize BMP585: " + sensor.status_string())

print("BMP585 connected!")

# Configure the interrupt pin as push/pull, active high, pulsed
sensor.configure_interrupt(BMP5_PULSED, BMP5_ACTIVE_HIGH, BMP5_INTR_PUSH_PULL, True)

# Only assert the pin on a new data-ready reading
sensor.set_interrupt_source(True)

interrupt_pin = Pin(INT_PIN, Pin.IN)
interrupt_pin.irq(trigger=Pin.IRQ_RISING, handler=int_callback)

while True:
    if data_ready:
        data_ready = False
        status = sensor.get_interrupt_status()
        if status is not None and (status & BMP5_INT_ASSERTED_DRDY):
            if sensor.get_sensor_data() == BMP5_OK:
                print(
                    "Pressure: {:.2f} Pa\tTemperature: {:.2f} degC".format(
                        sensor.data.pressure, sensor.data.temperature
                    )
                )
            else:
                print("Failed to read sensor data:", sensor.status_string())
