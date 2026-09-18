# Soldered BMP585 Barometric Pressure Sensor MicroPython Library

| ![Soldered BMP585 Barometric Pressure Sensor breakout](TODO_PRODUCT_IMAGE_URL) |
| :-----------------------------------------------------------------------------------------------: |
|                          [Soldered BMP585 Barometric Pressure Sensor breakout](https://www.solde.red/333189)                     |

<!-- TODO: product not released yet (SKU 333189), swap the image URL above once the listing is live -->

Breakout board for the Bosch BMP585 barometric pressure sensor, measuring absolute pressure with ±6 Pa relative accuracy at output data rates up to 240 Hz, with configurable oversampling and an on-chip IIR filter. It supports normal, forced and continuous power modes for balancing measurement speed against power consumption. The board communicates over I2C only and is part of the [Qwiic ecosystem](https://soldered.com/collections/qwiic-ecosystem).

### Quick start

```python
from bmp585 import BMP585, BMP5_OK
import time

sensor = BMP585()  # Or BMP585(address=BMP5_I2C_ADDR_PRIM) if the SDO pin is pulled low

while True:
    if sensor.get_sensor_data() == BMP5_OK:
        print(sensor.data.pressure, sensor.data.temperature)
    time.sleep(1)
```

Have a look at the scripts in `Examples/` for basic readings, forced-mode single-shot readings with a custom configuration, and using the sensor's physical interrupt pin.

### How to install

Use [mim](https://checkmim.com/packages).

or

After [**installing the mpremote package**](https://docs.micropython.org/en/latest/reference/mpremote.html), install the library on your board using the following command:

```sh
  mpremote mip install github:SolderedElectronics/Soldered-BMP585-MicroPython-Library
```
Or, if you're running a Windows OS:

```sh
  python -m mpremote mip install github:SolderedElectronics/Soldered-BMP585-MicroPython-Library
```

### Repository Contents

- **bmp585.py** - MicroPython driver class, I2C only
- **package.json** - mip install manifest
- **/Examples** - examples for basic readings, forced-mode custom configuration, and the physical interrupt pin

### Examples

| Example | What it does |
| :------ | :----------- |
| `bmp585-basicReadings.py` | Reads pressure and temperature in a loop in normal power mode, the mode most applications want |
| `bmp585-forcedModeCustomConfig.py` | Configures oversampling/IIR filtering and takes single-shot readings in forced power mode, for low-power, infrequent-reading use cases |
| `bmp585-dataReadyInterrupt.py` | Uses the sensor's physical interrupt pin to know when a new reading is ready, instead of polling the interrupt status register |

### Hardware design

You can find hardware design for this board in _Soldered BMP585 Barometric Pressure Sensor breakout_ hardware repository.

### Documentation

Access library documentation [here](https://docs.soldered.com/).

### About Soldered

![Soldered Logo](https://raw.githubusercontent.com/SolderedElectronics/Soldered-Generic-Arduino-Library/dev/extras/Soldered-logo-color.png)

At Soldered, we design and manufacture a wide selection of electronic products to help you turn your ideas into acts and bring you one step closer to your final project. Our products are intented for makers and crafted in-house by our experienced team in Osijek, Croatia. We believe that sharing is a crucial element for improvement and innovation, and we work hard to stay connected with all our makers regardless of their skill or experience level. Therefore, all our products are open-source. Finally, we always have your back. If you face any problem concerning either your shopping experience or your electronics project, our team will help you deal with it, offering efficient customer service and cost-free technical support anytime. Some of those might be useful for you:

- [Web Store](https://www.soldered.com/shop)
- [Tutorials & Projects](https://soldered.com/learn)
- [Documentation](https://docs.soldered.com)

### Original source

This library is a register-level port of the [BMP5_SensorAPI](https://github.com/boschsensortec/BMP5_SensorAPI) by Bosch Sensortec, cross-checked against the Soldered [Arduino](https://github.com/SolderedElectronics/Soldered-BMP585-Arduino-Library) and [ESP-IDF](https://github.com/SolderedElectronics/Soldered-BMP585-ESP-IDF-Component) BMP585 libraries. Thank you, Bosch Sensortec.

### Open-source license

Soldered invests vast amounts of time into hardware & software for these products, which are all open-source. Please support future development by buying one of our products.

Check license details in the LICENSE file. Long story short, use these open-source files for any purpose you want to, as long as you apply the same open-source licence to it and disclose the original source. No warranty - all designs in this repository are distributed in the hope that they will be useful, but without any warranty. They are provided "AS IS", therefore without warranty of any kind, either expressed or implied. The entire quality and performance of what you do with the contents of this repository are your responsibility. In no event, Soldered (TAVU) will be liable for your damages, losses, including any general, special, incidental or consequential damage arising out of the use or inability to use the contents of this repository.

## Have fun!

And thank you from your fellow makers at Soldered Electronics.
