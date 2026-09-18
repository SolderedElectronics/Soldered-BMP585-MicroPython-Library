# FILE: bmp585.py
# AUTHOR: Soldered Electronics
# BRIEF: MicroPython driver for the Bosch BMP585 barometric pressure sensor,
#        register-level ported from Bosch's BMP5_SensorAPI (bmp5.c/bmp5_defs.h,
#        BSD-3-Clause), matching the Soldered BMP585 Arduino/ESP-IDF libraries
# LAST UPDATED: 2026-09-18

from machine import I2C, Pin
from os import uname
import time

# I2C addresses
BMP5_I2C_ADDR_PRIM = 0x46
BMP5_I2C_ADDR_SEC = 0x47

# Chip identifiers and command bytes
BMP5_CHIP_ID_PRIM = 0x50
BMP5_CHIP_ID_SEC = 0x51
BMP5_SOFT_RESET_CMD = 0xB6

# Return codes of the driver
BMP5_OK = 0
BMP5_E_NULL_PTR = -1
BMP5_E_COM_FAIL = -2
BMP5_E_DEV_NOT_FOUND = -3
BMP5_E_INVALID_CHIP_ID = -4
BMP5_E_NVM_NOT_READY = -5
BMP5_E_POR_SOFTRESET = -6
BMP5_E_INVALID_POWERMODE = -7

# Aggregated result of check_status()
BMP5_ERROR = -1

# Enable / disable
BMP5_DISABLE = 0
BMP5_ENABLE = 1

# Power mode configurations
BMP5_POWERMODE_STANDBY = 0
BMP5_POWERMODE_NORMAL = 1
BMP5_POWERMODE_FORCED = 2
BMP5_POWERMODE_CONTINUOUS = 3
BMP5_POWERMODE_DEEP_STANDBY = 4

# Output data rate configurations
BMP5_ODR_240_HZ = 0x00
BMP5_ODR_218_5_HZ = 0x01
BMP5_ODR_199_1_HZ = 0x02
BMP5_ODR_179_2_HZ = 0x03
BMP5_ODR_160_HZ = 0x04
BMP5_ODR_149_3_HZ = 0x05
BMP5_ODR_140_HZ = 0x06
BMP5_ODR_129_8_HZ = 0x07
BMP5_ODR_120_HZ = 0x08
BMP5_ODR_110_1_HZ = 0x09
BMP5_ODR_100_2_HZ = 0x0A
BMP5_ODR_89_6_HZ = 0x0B
BMP5_ODR_80_HZ = 0x0C
BMP5_ODR_70_HZ = 0x0D
BMP5_ODR_60_HZ = 0x0E
BMP5_ODR_50_HZ = 0x0F
BMP5_ODR_45_HZ = 0x10
BMP5_ODR_40_HZ = 0x11
BMP5_ODR_35_HZ = 0x12
BMP5_ODR_30_HZ = 0x13
BMP5_ODR_25_HZ = 0x14
BMP5_ODR_20_HZ = 0x15
BMP5_ODR_15_HZ = 0x16
BMP5_ODR_10_HZ = 0x17
BMP5_ODR_05_HZ = 0x18
BMP5_ODR_04_HZ = 0x19
BMP5_ODR_03_HZ = 0x1A
BMP5_ODR_02_HZ = 0x1B
BMP5_ODR_01_HZ = 0x1C
BMP5_ODR_0_5_HZ = 0x1D
BMP5_ODR_0_250_HZ = 0x1E
BMP5_ODR_0_125_HZ = 0x1F

# Oversampling settings, used for both temperature and pressure
BMP5_OVERSAMPLING_1X = 0x00
BMP5_OVERSAMPLING_2X = 0x01
BMP5_OVERSAMPLING_4X = 0x02
BMP5_OVERSAMPLING_8X = 0x03
BMP5_OVERSAMPLING_16X = 0x04
BMP5_OVERSAMPLING_32X = 0x05
BMP5_OVERSAMPLING_64X = 0x06
BMP5_OVERSAMPLING_128X = 0x07

# IIR filter coefficients, used for both temperature and pressure
BMP5_IIR_FILTER_BYPASS = 0x00
BMP5_IIR_FILTER_COEFF_1 = 0x01
BMP5_IIR_FILTER_COEFF_3 = 0x02
BMP5_IIR_FILTER_COEFF_7 = 0x03
BMP5_IIR_FILTER_COEFF_15 = 0x04
BMP5_IIR_FILTER_COEFF_31 = 0x05
BMP5_IIR_FILTER_COEFF_63 = 0x06
BMP5_IIR_FILTER_COEFF_127 = 0x07

# Interrupt assertion status flags, as returned by get_interrupt_status()
BMP5_INT_ASSERTED_DRDY = 0x01
BMP5_INT_ASSERTED_FIFO_FULL = 0x02
BMP5_INT_ASSERTED_FIFO_THRES = 0x04
BMP5_INT_ASSERTED_PRESSURE_OOR = 0x08
BMP5_INT_ASSERTED_POR_SOFTRESET_COMPLETE = 0x10

# NVM status flags, used internally by _init_sensor() only
_BMP5_INT_NVM_RDY = 0x02
_BMP5_INT_NVM_ERR = 0x04

# Register map
BMP5_REG_CHIP_ID = 0x01
BMP5_REG_REV_ID = 0x02
BMP5_REG_CHIP_STATUS = 0x11
BMP5_REG_TEMP_DATA_XLSB = 0x1D
BMP5_REG_PRESS_DATA_XLSB = 0x20
BMP5_REG_INT_STATUS = 0x27
BMP5_REG_STATUS = 0x28
BMP5_REG_NVM_ADDR = 0x2B
BMP5_REG_DSP_CONFIG = 0x30
BMP5_REG_DSP_IIR = 0x31
BMP5_REG_FIFO_SEL = 0x18
BMP5_REG_OSR_CONFIG = 0x36
BMP5_REG_ODR_CONFIG = 0x37
BMP5_REG_CMD = 0x7E

# Bit masks / positions used to pack and unpack the registers above. Named to
# match the Bosch driver's macros (bmp5_defs.h) so they can be cross-checked
# against it directly.
_BMP5_TEMP_OS_MSK = 0x07
_BMP5_PRESS_OS_MSK = 0x38
_BMP5_PRESS_OS_POS = 3
_BMP5_PRESS_EN_MSK = 0x40
_BMP5_PRESS_EN_POS = 6
_BMP5_ODR_MSK = 0x7C
_BMP5_ODR_POS = 2
_BMP5_POWERMODE_MSK = 0x03
_BMP5_DEEP_DISABLE_MSK = 0x80
_BMP5_DEEP_DISABLE_POS = 7
_BMP5_DEEP_ENABLED = 0
_BMP5_DEEP_DISABLED = 1
_BMP5_SET_IIR_TEMP_MSK = 0x07
_BMP5_SET_IIR_PRESS_MSK = 0x38
_BMP5_SET_IIR_PRESS_POS = 3
_BMP5_SHDW_SET_IIR_TEMP_MSK = 0x08
_BMP5_SHDW_SET_IIR_TEMP_POS = 3
_BMP5_SHDW_SET_IIR_PRESS_MSK = 0x20
_BMP5_SHDW_SET_IIR_PRESS_POS = 5
_BMP5_IIR_FLUSH_FORCED_EN_MSK = 0x04
_BMP5_IIR_FLUSH_FORCED_EN_POS = 2
_BMP5_IIR_BYPASS_MSK = 0xC0
_BMP5_FIFO_FRAME_SEL_MSK = 0x03

# Delays required by the sensor, in microseconds
_BMP5_DELAY_US_SOFT_RESET = 2000
_BMP5_DELAY_US_STANDBY = 2500

# Text descriptions of the status codes, used by status_string()
_STATUS_STRINGS = {
    BMP5_OK: "",
    BMP5_E_NULL_PTR: "Null pointer",
    BMP5_E_COM_FAIL: "Communication failure",
    BMP5_E_DEV_NOT_FOUND: "Sensor not found",
    BMP5_E_INVALID_CHIP_ID: "Invalid chip id",
    BMP5_E_NVM_NOT_READY: "NVM not ready",
    BMP5_E_POR_SOFTRESET: "Power-on reset/softreset failure",
    BMP5_E_INVALID_POWERMODE: "Invalid powermode",
}


class BMP585Data:
    """One pressure/temperature reading."""

    def __init__(self):
        self.pressure = 0.0
        self.temperature = 0.0

    def __repr__(self):
        return "BMP585Data(pressure={:.2f}, temperature={:.2f})".format(
            self.pressure, self.temperature
        )


class BMP585:
    """
    MicroPython driver for the Soldered BMP585 breakout board, I2C only.

    The sensor is initialized by the constructor, which raises an exception
    when it cannot be reached. Every other method stores its result in the
    status attribute instead of raising, the same way the Arduino/ESP-IDF
    libraries do, so check_status() and status_string() report what went
    wrong.
    """

    def __init__(self, i2c=None, address=BMP5_I2C_ADDR_SEC):
        """
        Initialize the BMP585.

        :param i2c: Initialized I2C object, auto-detected on known boards
        :param address: I2C address, BMP5_I2C_ADDR_SEC (0x47) by default
        """
        if i2c is not None:
            self.i2c = i2c
        else:
            if uname().sysname in (
                "esp32",
                "esp8266",
                "Soldered Dasduino CONNECTPLUS",
            ):
                self.i2c = I2C(0, scl=Pin(22), sda=Pin(21))
            else:
                raise Exception("Board not recognized, enter I2C pins manually")

        self.address = address
        self.status = BMP5_OK
        self.intf_rslt = BMP5_OK
        self.chip_id = 0
        self.data = BMP585Data()

        self._init_sensor()

    # ------------------------------------------------------------------
    # Bus access
    # ------------------------------------------------------------------

    def _get_regs(self, reg_addr, length):
        """Read a block of registers, raises OSError on a bus failure."""
        data = self.i2c.readfrom_mem(self.address, reg_addr, length)
        self.intf_rslt = BMP5_OK
        return data

    def _set_regs(self, reg_addr, reg_data):
        """
        Write a block of sequential registers.

        Unlike the BMA400, the BMP585 does support I2C burst writes (see
        bmp5_set_regs() in bmp5.c: the SPI path splits into single-byte
        writes, but the I2C/I3C path passes the whole block to dev->write()
        in one call), so a single writeto_mem() call is correct here.
        """
        self.i2c.writeto_mem(self.address, reg_addr, bytes(reg_data))
        self.intf_rslt = BMP5_OK

    def read_reg(self, reg_addr, length=1):
        """
        Read one or more registers.

        :param reg_addr: Address of the first register
        :param length: Number of bytes to read
        :return: An integer for a single byte, bytes otherwise, None on error
        """
        try:
            data = self._get_regs(reg_addr, length)
            self.status = BMP5_OK
            return data[0] if length == 1 else data
        except OSError:
            self.intf_rslt = BMP5_E_COM_FAIL
            self.status = BMP5_E_COM_FAIL
            return None

    def read_regs(self, reg_addr, length):
        """Alias of read_reg(), kept for parity with the Arduino/ESP-IDF ports."""
        return self.read_reg(reg_addr, length)

    def write_reg(self, reg_addr, reg_data):
        """
        Write one or more registers.

        :param reg_addr: Address of the first register
        :param reg_data: Single byte, or a list/bytes of sequential data
        """
        if isinstance(reg_data, int):
            reg_data = [reg_data]

        try:
            self._set_regs(reg_addr, reg_data)
            self.status = BMP5_OK
        except OSError:
            self.intf_rslt = BMP5_E_COM_FAIL
            self.status = BMP5_E_COM_FAIL

    def write_regs(self, reg_addr, reg_data):
        """Alias of write_reg(), kept for parity with the Arduino/ESP-IDF ports."""
        self.write_reg(reg_addr, reg_data)

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

    def _soft_reset_raw(self):
        self._set_regs(BMP5_REG_CMD, [BMP5_SOFT_RESET_CMD])
        time.sleep_us(_BMP5_DELAY_US_SOFT_RESET)

        por_status = self._get_regs(BMP5_REG_INT_STATUS, 1)[0]
        if not (por_status & BMP5_INT_ASSERTED_POR_SOFTRESET_COMPLETE):
            raise Exception("BMP585 did not confirm soft reset completion")

    def _init_sensor(self):
        """Reset the sensor, check its chip ID and NVM status."""
        try:
            self._soft_reset_raw()
        except OSError:
            self.intf_rslt = BMP5_E_COM_FAIL
            self.status = BMP5_E_COM_FAIL
            raise Exception(
                "BMP585 not responding on address 0x{:02x}".format(self.address)
            )
        except Exception:
            self.status = BMP5_E_POR_SOFTRESET
            raise

        try:
            self.chip_id = self._get_regs(BMP5_REG_CHIP_ID, 1)[0]
        except OSError:
            self.intf_rslt = BMP5_E_COM_FAIL
            self.status = BMP5_E_COM_FAIL
            raise Exception(
                "BMP585 not responding on address 0x{:02x}".format(self.address)
            )

        if self.chip_id not in (BMP5_CHIP_ID_PRIM, BMP5_CHIP_ID_SEC):
            self.status = BMP5_E_INVALID_CHIP_ID
            raise Exception(
                "BMP585 not found, chip ID 0x{:02x} was read instead of "
                "0x{:02x}/0x{:02x}".format(self.chip_id, BMP5_CHIP_ID_PRIM, BMP5_CHIP_ID_SEC)
            )

        try:
            nvm_status = self._get_regs(BMP5_REG_STATUS, 1)[0]
        except OSError:
            self.intf_rslt = BMP5_E_COM_FAIL
            self.status = BMP5_E_COM_FAIL
            raise Exception(
                "BMP585 not responding on address 0x{:02x}".format(self.address)
            )

        if not (nvm_status & _BMP5_INT_NVM_RDY) or (nvm_status & _BMP5_INT_NVM_ERR):
            self.status = BMP5_E_NVM_NOT_READY
            raise Exception("BMP585 NVM not ready")

        try:
            self._set_osr_odr_config_raw(
                {
                    "osr_t": BMP5_OVERSAMPLING_64X,
                    "osr_p": BMP5_OVERSAMPLING_4X,
                    "odr": BMP5_ODR_50_HZ,
                    "press_en": BMP5_ENABLE,
                }
            )
            self._set_power_mode_full(BMP5_POWERMODE_NORMAL)
            self.status = BMP5_OK
        except OSError:
            self.intf_rslt = BMP5_E_COM_FAIL
            self.status = BMP5_E_COM_FAIL
            raise Exception(
                "BMP585 not responding on address 0x{:02x}".format(self.address)
            )

    def soft_reset(self):
        """Soft reset the sensor, the configuration is lost."""
        try:
            self._soft_reset_raw()
            self.status = BMP5_OK
        except OSError:
            self.intf_rslt = BMP5_E_COM_FAIL
            self.status = BMP5_E_COM_FAIL
        except Exception:
            self.status = BMP5_E_POR_SOFTRESET

    # ------------------------------------------------------------------
    # Power mode
    #
    # STANDBY, NORMAL, FORCED and CONTINUOUS all live in the same 2 bit field
    # of ODR_CONFIG, but the sensor must be put into STANDBY first before
    # switching to any of the other three (see bmp5.c's set_power_mode()).
    # DEEP_STANDBY is a separate state entered/exited through its own
    # sequence (see _set_deep_standby_mode_raw()).
    # ------------------------------------------------------------------

    def _set_power_mode_raw(self, mode):
        """Direct port of bmp5.c's static set_power_mode(): one register write."""
        reg_data = self._get_regs(BMP5_REG_ODR_CONFIG, 1)[0]
        reg_data = (reg_data & ~_BMP5_DEEP_DISABLE_MSK & 0xFF) | (
            (_BMP5_DEEP_DISABLED << _BMP5_DEEP_DISABLE_POS) & _BMP5_DEEP_DISABLE_MSK
        )
        reg_data = (reg_data & ~_BMP5_POWERMODE_MSK & 0xFF) | (mode & _BMP5_POWERMODE_MSK)
        self._set_regs(BMP5_REG_ODR_CONFIG, [reg_data])

    def _set_deep_standby_mode_raw(self):
        """Direct port of bmp5.c's static set_deep_standby_mode()."""
        reg_data = self._get_regs(BMP5_REG_ODR_CONFIG, 1)[0]
        reg_data = reg_data & ~_BMP5_DEEP_DISABLE_MSK & 0xFF  # deep_dis = 0 (enabled)
        reg_data = (reg_data & ~_BMP5_ODR_MSK & 0xFF) | (
            (BMP5_ODR_01_HZ << _BMP5_ODR_POS) & _BMP5_ODR_MSK
        )
        self._set_regs(BMP5_REG_ODR_CONFIG, [reg_data])

        iir_reg = self._get_regs(BMP5_REG_DSP_IIR, 1)[0]
        self._set_regs(BMP5_REG_DSP_IIR, [iir_reg & _BMP5_IIR_BYPASS_MSK])

        fifo_sel = self._get_regs(BMP5_REG_FIFO_SEL, 1)[0]
        self._set_regs(BMP5_REG_FIFO_SEL, [fifo_sel & ~_BMP5_FIFO_FRAME_SEL_MSK & 0xFF])

    def _get_power_mode_raw(self):
        """Direct port of bmp5.c's bmp5_get_power_mode()."""
        reg_data = self._get_regs(BMP5_REG_ODR_CONFIG, 1)[0]
        pwrmode = reg_data & _BMP5_POWERMODE_MSK

        if pwrmode == BMP5_POWERMODE_STANDBY:
            deep_dis = (reg_data & _BMP5_DEEP_DISABLE_MSK) >> _BMP5_DEEP_DISABLE_POS
            if deep_dis == _BMP5_DEEP_ENABLED:
                if self._check_deepstandby_raw():
                    return BMP5_POWERMODE_DEEP_STANDBY

        return pwrmode

    def _check_deepstandby_raw(self):
        """
        Direct port of bmp5.c's static check_deepstandby_mode(): whether the
        current config, combined with deep_dis being enabled, means the
        sensor is actually sitting in deep standby rather than plain standby.
        """
        fifo_frame_sel = self._get_regs(BMP5_REG_FIFO_SEL, 1)[0] & _BMP5_FIFO_FRAME_SEL_MSK
        osr_odr = self._get_osr_odr_config_raw()
        iir = self._get_iir_config_raw()

        return (
            osr_odr["odr"] > BMP5_ODR_05_HZ
            and fifo_frame_sel == BMP5_DISABLE
            and iir["set_iir_t"] == BMP5_IIR_FILTER_BYPASS
            and iir["set_iir_p"] == BMP5_IIR_FILTER_BYPASS
        )

    def _exit_deep_standby_raw(self):
        """Direct port of bmp5.c's static set_standby_mode()."""
        if self._get_power_mode_raw() == BMP5_POWERMODE_DEEP_STANDBY:
            self._set_power_mode_full(BMP5_POWERMODE_STANDBY)

    def _set_power_mode_full(self, mode):
        """Direct port of bmp5.c's bmp5_set_power_mode()."""
        last_mode = self._get_power_mode_raw()

        if last_mode != BMP5_POWERMODE_STANDBY:
            self._set_power_mode_raw(BMP5_POWERMODE_STANDBY)
            time.sleep_us(_BMP5_DELAY_US_STANDBY)

        if mode == BMP5_POWERMODE_DEEP_STANDBY:
            self._set_deep_standby_mode_raw()
        elif mode == BMP5_POWERMODE_STANDBY:
            pass
        elif mode in (BMP5_POWERMODE_NORMAL, BMP5_POWERMODE_FORCED, BMP5_POWERMODE_CONTINUOUS):
            self._set_power_mode_raw(mode)
        else:
            raise ValueError("invalid power mode")

    def set_mode(self, mode):
        """
        Set the power mode.

        :param mode: BMP5_POWERMODE_STANDBY, BMP5_POWERMODE_NORMAL,
                      BMP5_POWERMODE_FORCED, BMP5_POWERMODE_CONTINUOUS or
                      BMP5_POWERMODE_DEEP_STANDBY
        """
        try:
            self._set_power_mode_full(mode)
            self.status = BMP5_OK
        except ValueError:
            self.status = BMP5_E_INVALID_POWERMODE
        except OSError:
            self.intf_rslt = BMP5_E_COM_FAIL
            self.status = BMP5_E_COM_FAIL

    def get_mode(self):
        """
        Get the power mode the sensor is actually running in.

        :return: BMP5_POWERMODE_STANDBY, BMP5_POWERMODE_NORMAL,
                 BMP5_POWERMODE_FORCED, BMP5_POWERMODE_CONTINUOUS or
                 BMP5_POWERMODE_DEEP_STANDBY, None on error
        """
        try:
            mode = self._get_power_mode_raw()
            self.status = BMP5_OK
            return mode
        except OSError:
            self.intf_rslt = BMP5_E_COM_FAIL
            self.status = BMP5_E_COM_FAIL
            return None

    # ------------------------------------------------------------------
    # OSR / ODR / pressure-enable configuration
    #
    # These share one 2 byte block (OSR_CONFIG, ODR_CONFIG) with the
    # powermode/deep_disable bits, so every setter reads the block first and
    # only touches its own bits, to avoid clobbering the power mode.
    # ------------------------------------------------------------------

    def _get_osr_odr_config_raw(self):
        data = self._get_regs(BMP5_REG_OSR_CONFIG, 2)
        return {
            "osr_t": data[0] & _BMP5_TEMP_OS_MSK,
            "osr_p": (data[0] & _BMP5_PRESS_OS_MSK) >> _BMP5_PRESS_OS_POS,
            "press_en": (data[0] & _BMP5_PRESS_EN_MSK) >> _BMP5_PRESS_EN_POS,
            "odr": (data[1] & _BMP5_ODR_MSK) >> _BMP5_ODR_POS,
        }

    def _set_osr_odr_config_raw(self, config):
        data = bytearray(self._get_regs(BMP5_REG_OSR_CONFIG, 2))
        data[0] = (data[0] & ~_BMP5_TEMP_OS_MSK & 0xFF) | (config["osr_t"] & _BMP5_TEMP_OS_MSK)
        data[0] = (data[0] & ~_BMP5_PRESS_OS_MSK & 0xFF) | (
            (config["osr_p"] << _BMP5_PRESS_OS_POS) & _BMP5_PRESS_OS_MSK
        )
        data[0] = (data[0] & ~_BMP5_PRESS_EN_MSK & 0xFF) | (
            (config["press_en"] << _BMP5_PRESS_EN_POS) & _BMP5_PRESS_EN_MSK
        )
        data[1] = (data[1] & ~_BMP5_ODR_MSK & 0xFF) | ((config["odr"] << _BMP5_ODR_POS) & _BMP5_ODR_MSK)
        self._set_regs(BMP5_REG_OSR_CONFIG, data)

    def set_osr_odr_config(self, osr_t, osr_p, odr, press_en=True):
        """
        Set oversampling, output data rate and pressure-enable configuration.

        :param osr_t: Temperature oversampling, one of the BMP5_OVERSAMPLING_* constants
        :param osr_p: Pressure oversampling, one of the BMP5_OVERSAMPLING_* constants
        :param odr: Output data rate, one of the BMP5_ODR_* constants
        :param press_en: Whether to enable pressure measurement
        """
        try:
            self._set_osr_odr_config_raw(
                {
                    "osr_t": osr_t,
                    "osr_p": osr_p,
                    "odr": odr,
                    "press_en": BMP5_ENABLE if press_en else BMP5_DISABLE,
                }
            )
            self.status = BMP5_OK
        except OSError:
            self.intf_rslt = BMP5_E_COM_FAIL
            self.status = BMP5_E_COM_FAIL

    def get_osr_odr_config(self):
        """
        Get the current oversampling, output data rate and pressure-enable configuration.

        :return: dict with keys osr_t, osr_p, odr, press_en, None on error
        """
        try:
            config = self._get_osr_odr_config_raw()
            self.status = BMP5_OK
            return config
        except OSError:
            self.intf_rslt = BMP5_E_COM_FAIL
            self.status = BMP5_E_COM_FAIL
            return None

    # ------------------------------------------------------------------
    # IIR filter configuration
    #
    # Writable only in standby mode, so set_iir_config() steps the sensor
    # through standby and back, matching bmp5_set_iir_config().
    # ------------------------------------------------------------------

    def _get_iir_config_raw(self):
        data = self._get_regs(BMP5_REG_DSP_CONFIG, 2)
        return {
            "shdw_set_iir_t": (data[0] & _BMP5_SHDW_SET_IIR_TEMP_MSK) >> _BMP5_SHDW_SET_IIR_TEMP_POS,
            "shdw_set_iir_p": (data[0] & _BMP5_SHDW_SET_IIR_PRESS_MSK) >> _BMP5_SHDW_SET_IIR_PRESS_POS,
            "iir_flush_forced_en": (data[0] & _BMP5_IIR_FLUSH_FORCED_EN_MSK) >> _BMP5_IIR_FLUSH_FORCED_EN_POS,
            "set_iir_t": data[1] & _BMP5_SET_IIR_TEMP_MSK,
            "set_iir_p": (data[1] & _BMP5_SET_IIR_PRESS_MSK) >> _BMP5_SET_IIR_PRESS_POS,
        }

    def _set_iir_config_raw(self, config):
        data = bytearray(self._get_regs(BMP5_REG_DSP_CONFIG, 2))
        data[0] = (data[0] & ~_BMP5_SHDW_SET_IIR_TEMP_MSK & 0xFF) | (
            (config["shdw_set_iir_t"] << _BMP5_SHDW_SET_IIR_TEMP_POS) & _BMP5_SHDW_SET_IIR_TEMP_MSK
        )
        data[0] = (data[0] & ~_BMP5_SHDW_SET_IIR_PRESS_MSK & 0xFF) | (
            (config["shdw_set_iir_p"] << _BMP5_SHDW_SET_IIR_PRESS_POS) & _BMP5_SHDW_SET_IIR_PRESS_MSK
        )
        data[0] = (data[0] & ~_BMP5_IIR_FLUSH_FORCED_EN_MSK & 0xFF) | (
            (config["iir_flush_forced_en"] << _BMP5_IIR_FLUSH_FORCED_EN_POS) & _BMP5_IIR_FLUSH_FORCED_EN_MSK
        )
        data[1] = (data[1] & ~_BMP5_SET_IIR_TEMP_MSK & 0xFF) | (config["set_iir_t"] & _BMP5_SET_IIR_TEMP_MSK)
        data[1] = (data[1] & ~_BMP5_SET_IIR_PRESS_MSK & 0xFF) | (
            (config["set_iir_p"] << _BMP5_SET_IIR_PRESS_POS) & _BMP5_SET_IIR_PRESS_MSK
        )
        self._set_regs(BMP5_REG_DSP_CONFIG, data)

    def set_iir_config(self, iir_t, iir_p):
        """
        Set the IIR filter coefficient used for temperature and pressure data.

        :param iir_t: Temperature IIR coefficient, one of the BMP5_IIR_FILTER_* constants
        :param iir_p: Pressure IIR coefficient, one of the BMP5_IIR_FILTER_* constants
        """
        try:
            if iir_t != BMP5_IIR_FILTER_BYPASS or iir_p != BMP5_IIR_FILTER_BYPASS:
                self._exit_deep_standby_raw()

            curr_mode = self._get_power_mode_raw()
            if curr_mode != BMP5_POWERMODE_STANDBY:
                self._set_power_mode_full(BMP5_POWERMODE_STANDBY)

            self._set_iir_config_raw(
                {
                    "set_iir_t": iir_t,
                    "set_iir_p": iir_p,
                    "shdw_set_iir_t": BMP5_ENABLE,
                    "shdw_set_iir_p": BMP5_ENABLE,
                    "iir_flush_forced_en": BMP5_DISABLE,
                }
            )

            # IIR only works in standby mode, so a previous deep standby mode
            # is intentionally not restored here -- re-entering it would
            # reset the IIR settings straight back to bypass anyway.
            if curr_mode not in (BMP5_POWERMODE_STANDBY, BMP5_POWERMODE_DEEP_STANDBY):
                self._set_power_mode_full(curr_mode)

            self.status = BMP5_OK
        except OSError:
            self.intf_rslt = BMP5_E_COM_FAIL
            self.status = BMP5_E_COM_FAIL

    def get_iir_config(self):
        """
        Get the current IIR filter configuration.

        :return: dict with keys set_iir_t, set_iir_p, shdw_set_iir_t,
                 shdw_set_iir_p, iir_flush_forced_en, None on error
        """
        try:
            config = self._get_iir_config_raw()
            self.status = BMP5_OK
            return config
        except OSError:
            self.intf_rslt = BMP5_E_COM_FAIL
            self.status = BMP5_E_COM_FAIL
            return None

    # ------------------------------------------------------------------
    # Reading out measurements
    # ------------------------------------------------------------------

    def get_interrupt_status(self):
        """
        Get the data-ready / FIFO / OOR interrupt status.

        Useful for polling since this breakout doesn't expose the sensor's
        interrupt pin.

        :return: Bitmask of BMP5_INT_ASSERTED_* flags, None on error
        """
        try:
            status = self._get_regs(BMP5_REG_INT_STATUS, 1)[0]
            self.status = BMP5_OK
            return status
        except OSError:
            self.intf_rslt = BMP5_E_COM_FAIL
            self.status = BMP5_E_COM_FAIL
            return None

    def get_sensor_data(self):
        """
        Read a new pressure/temperature measurement into self.data.

        :return: BMP5_OK on success, an error code otherwise
        """
        try:
            press_en = self._get_osr_odr_config_raw()["press_en"]
            data = self._get_regs(BMP5_REG_TEMP_DATA_XLSB, 6)

            raw_t = data[0] | (data[1] << 8) | (data[2] << 16)
            if raw_t & 0x800000:
                raw_t -= 0x1000000
            self.data.temperature = raw_t / 65536.0

            if press_en == BMP5_ENABLE:
                raw_p = data[3] | (data[4] << 8) | (data[5] << 16)
                self.data.pressure = raw_p / 64.0
            else:
                self.data.pressure = 0.0

            self.status = BMP5_OK
        except OSError:
            self.intf_rslt = BMP5_E_COM_FAIL
            self.status = BMP5_E_COM_FAIL

        return self.status

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def check_status(self):
        """
        Check whether the last call failed.

        :return: BMP5_ERROR if the last call failed, BMP5_OK otherwise
        """
        if self.status < BMP5_OK:
            return BMP5_ERROR

        return BMP5_OK

    def status_string(self):
        """
        Get a short description of the current status code.

        :return: Description of the status, an empty string when it is OK
        """
        return _STATUS_STRINGS.get(self.status, "Undefined error code")
