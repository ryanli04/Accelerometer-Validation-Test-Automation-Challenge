"""
ADXL345 Accelerometer Test Script - Ryan Li

Notes:
- ADXL345 I2C address is 0x1D (ALT ADDRESS pin tied high).
- Pin 7 tied high to VDD so I2C mode
- TestBoard supplies: "2.5V" for VS, "1.8V" for VDD_IO.
- I2C pins are "MCU_DIO2" (SDA) and "MCU_DIO1" (SCL) at 400 kHz (Use fast mode for this for optimal performance).
- Configure ADXL345 in full-resolution, +/-16 g, 800 Hz ODR (0x0D).
- Using I²C at 400 kHz, **800 Hz ODR** is the max recommended; 1600/3200 Hz are only recommended with **SPI ≥ 2 MHz**.
- Sample during actuator motions and check constraints.
- Convert raw readings to g using full-resolution scale (~3.9 mg/LSB). Therefore g = raw_counts * 0.0039.
- Log results as specified.
"""

import time
import threading
import struct
import logging

from test_board_fwk import TestBoard, I2CError, ActuatorError  # provided by framework

# --------- Constants & logging setup ---------
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# ------------- ADXL345 specifics -------------
ADXL345_I2C_ADDR = 0x1D  # Since SDO/ALT ADDRESS pin is connected to 1.8V (high)

# ADXL345 register addresses
REG_DEVID      = 0x00
REG_BW_RATE    = 0x2C
REG_POWER_CTL  = 0x2D
REG_DATA_FORMAT= 0x31
REG_DATAX0     = 0x32  # X0..Z1 are 0x32-0x37

# Constants from datasheet (full-resolution mode)
FULL_RES_SCALE_G_PER_LSB = 0.0039  # ~4 mg/LSB

# Self-test expected ranges in g (Table 1)
SELFTEST_X_MIN_G = 0.20
SELFTEST_X_MAX_G = 2.10
SELFTEST_Y_MIN_G = -2.10
SELFTEST_Y_MAX_G = -0.20
SELFTEST_Z_MIN_G = 0.30
SELFTEST_Z_MAX_G = 3.40

# --------- ADXL345 Driver Class ---------
class ADXL345:
    def __init__(self, board: TestBoard, addr: int = ADXL345_I2C_ADDR): # Initialize with board and I2C address.
        self.board = board
        self.addr = addr

    # --------- Low-level I2C helpers ---------

    def _write_reg(self, reg: int, value: int):
        #Write a single byte to a register.
        self.board.i2c_cmd(self.addr, [reg & 0xFF, value & 0xFF], resp_len=0)

    def _read_bytes(self, start_reg: int, length: int) -> bytes:
        #Read 'length' bytes starting at start_reg.
        resp = self.board.i2c_cmd(self.addr, [start_reg & 0xFF], resp_len=length)
        return bytes(resp)

    def _read_reg(self, reg: int) -> int:
        #Read a single register.
        return self._read_bytes(reg, 1)[0]

    # --------- Configuration & sanity checks ---------

    def check_devid(self):
        #Verify device ID is 0xE5.
        devid = self._read_reg(REG_DEVID)
        if devid != 0xE5:
            raise RuntimeError(f"Unexpected DEVID 0x{devid:02X}, expected 0xE5")

    def configure_for_max_rate(self):
        # Configure:
        # - ODR = 800 Hz (BW_RATE = 0x0D)
        # - Full-resolution, +/-16 g (DATA_FORMAT = FULL_RES=1, range=0b11)
        # - Measurement mode (POWER_CTL: MEASURE=1)
        # BW_RATE: 0x0D -> 800 Hz ODR
        self._write_reg(REG_BW_RATE, 0x0D)

        # DATA_FORMAT:
        # Bit7 SELF_TEST = 0
        # Bit6 SPI       = 0 (don't care for I2C)
        # Bit5 INT_INVERT= 0 (active high)
        # Bit4 reserved  = 0
        # Bit3 FULL_RES  = 1
        # Bit2 JUSTIFY   = 0 (right-justified)
        # Bits1:0 RANGE  = 0b11 (+/-16 g)
        data_format = (1 << 3) | 0x03  # 0b00001000 | 0b11 = 0x0B
        self._write_reg(REG_DATA_FORMAT, data_format)

        # POWER_CTL:
        # Bit3 MEASURE = 1, others 0 => 0x08 (0b00001000)
        self._write_reg(REG_POWER_CTL, 0x08)

        # Give device a moment to settle
        time.sleep(0.01)

    # --------- Data reading ---------

    def read_raw_xyz(self):
        #
        # Read raw X, Y, Z as signed ints from data registers.
        data = self._read_bytes(REG_DATAX0, 6)
        # little-endian 16-bit signed for X, Y, Z (LSB first)
        x, y, z = struct.unpack("<hhh", data)
        return x, y, z

    def read_g_xyz(self):
        # Read X, Y, Z in g units using full-resolution scale (~3.9 mg/LSB).
        x_raw, y_raw, z_raw = self.read_raw_xyz()
        scale = FULL_RES_SCALE_G_PER_LSB
        return (
            x_raw * scale,
            y_raw * scale,
            z_raw * scale,
        )

    # --------- Self-test ---------

    def _set_self_test(self, enable: bool):
        # Set or clear SELF_TEST bit in DATA_FORMAT.
        fmt = self._read_reg(REG_DATA_FORMAT)
        if enable:
            fmt |= (1 << 7)
        else:
            fmt &= ~(1 << 7)
        self._write_reg(REG_DATA_FORMAT, fmt)
        # allow 4 * tau to settle; tau ~ 1/800 s; 10 ms is safe
        time.sleep(0.01)

    def _avg_samples_g(self, n_samples: int = 32, delay_s: float = 0.002):
        # Average N samples in g. delay_s controls time between samples.
        sum_x = sum_y = sum_z = 0.0
        for _ in range(n_samples):
            gx, gy, gz = self.read_g_xyz()
            sum_x += gx
            sum_y += gy
            sum_z += gz
            time.sleep(delay_s)
        return sum_x / n_samples, sum_y / n_samples, sum_z / n_samples

    def run_self_test(self):
        # Execute self-test and assert that deltas are within datasheet ranges.
        # Baseline (self-test off)
        self._set_self_test(False)
        base_x, base_y, base_z = self._avg_samples_g()

        # Self-test on
        self._set_self_test(True)
        st_x, st_y, st_z = self._avg_samples_g()

        # Turn self-test off again
        self._set_self_test(False)

        dx = st_x - base_x
        dy = st_y - base_y
        dz = st_z - base_z

        # Check against datasheet ranges (Table 1)
        if not (SELFTEST_X_MIN_G <= dx <= SELFTEST_X_MAX_G):
            raise RuntimeError(
                f"Self-test X delta out of range: {dx:.3f} g "
                f"(expected {SELFTEST_X_MIN_G}..{SELFTEST_X_MAX_G} g)"
            )
        if not (SELFTEST_Y_MIN_G <= dy <= SELFTEST_Y_MAX_G):
            raise RuntimeError(
                f"Self-test Y delta out of range: {dy:.3f} g "
                f"(expected {SELFTEST_Y_MIN_G}..{SELFTEST_Y_MAX_G} g)"
            )
        if not (SELFTEST_Z_MIN_G <= dz <= SELFTEST_Z_MAX_G):
            raise RuntimeError(
                f"Self-test Z delta out of range: {dz:.3f} g "
                f"(expected {SELFTEST_Z_MIN_G}..{SELFTEST_Z_MAX_G} g)"
            )
        
        # If all checks pass, return silently.

# --------- Motion check helpers ---------

def _run_motion_with_checks(board: TestBoard, accel: ADXL345,
                            config_name: str, check_fn,
                            sample_period_s: float = 0.01):
    # Run actuator_move(config_name) in a background thread and, while it runs,
    # repeatedly sample accelerometer and apply check_fn(gx, gy, gz).

    # check_fn should raise an exception if a constraint is violated.
    def move_thread():
        board.actuator_move(config_name)

    t = threading.Thread(target=move_thread, daemon=True)
    t.start()

    # Sample while motion is in progress
    while t.is_alive():
        gx, gy, gz = accel.read_g_xyz()
        check_fn(gx, gy, gz)
        time.sleep(sample_period_s)

    # Take one final sample at the end (in case final orientation matters)
    gx, gy, gz = accel.read_g_xyz()
    check_fn(gx, gy, gz)

# --------- Specific motion check functions ---------

def check_slow_climb(gx: float, gy: float, gz: float):
    # y between -1 g and 1 g, z between 6 g and 8 g
    if not (-1.0 <= gy <= 1.0):
        raise RuntimeError(f"slow_climb: Y out of range: {gy:.2f} g (expected -1..1 g)")
    if not (6.0 <= gz <= 8.0):
        raise RuntimeError(f"slow_climb: Z out of range: {gz:.2f} g (expected 6..8 g)")


def check_sharp_turn(gx: float, gy: float, gz: float):
    # x > 5 g and y > 5 g
    if not (gx > 5.0):
        raise RuntimeError(f"sharp_turn: X too low: {gx:.2f} g (expected > 5 g)")
    if not (gy > 5.0):
        raise RuntimeError(f"sharp_turn: Y too low: {gy:.2f} g (expected > 5 g)")


def check_quick_drop(gx: float, gy: float, gz: float):
    # z < -8 g
    if not (gz < -8.0):
        raise RuntimeError(f"quick_drop: Z too high: {gz:.2f} g (expected < -8 g)")

# --------- Top-level test entry point ---------

def run_test():
    start_time = time.time()
    test_passed = False
    failure_reason = ""
    board = None

    try:
        # Connect to test board
        board = TestBoard()

        # Turn on required power rails for ADXL345
        board.turn_on_ps("2V5")
        board.turn_on_ps("1V8")

        # Configure I2C i2c_setup(sda, scl, freq)
        board.i2c_setup("MCU_DIO2", "MCU_DIO1", 400000)

        # Create accelerometer driver
        accel = ADXL345(board)

        # Basic sanity check: DEVID
        accel.check_devid()

        # Configure at maximum valid ODR for I2C (800 Hz) and +/-16 g
        accel.configure_for_max_rate()

        # Run self-test
        accel.run_self_test()

        # Motion: slow_climb
        _run_motion_with_checks(board, accel, "slow_climb", check_slow_climb)

        # Motion: sharp_turn
        _run_motion_with_checks(board, accel, "sharp_turn", check_sharp_turn)

        # Motion: quick_drop
        _run_motion_with_checks(board, accel, "quick_drop", check_quick_drop)

        test_passed = True

    # Catch and log exceptions
    except ConnectionError as e:
        failure_reason = f"ConnectionError: {e}"
        logger.error(failure_reason)
    except I2CError as e:
        failure_reason = f"I2CError: {e}"
        logger.error(failure_reason)
    except ActuatorError as e:
        failure_reason = f"ActuatorError: {e}"
        logger.error(failure_reason)
    except RuntimeError as e:
        failure_reason = f"RuntimeError: {e}"
        logger.error(failure_reason)
    except Exception as e:
        failure_reason = f"Unexpected error: {type(e).__name__}: {e}"
        logger.error(failure_reason)

    finally:
        elapsed = time.time() - start_time

        # Try to power down gracefully
        if board is not None:
            try:
                board.turn_off_ps("2V5")
            except Exception as e:
                logger.warning(f"Error during power down 2V5: {e}")
            try:
                board.turn_off_ps("1V8")
            except Exception as e:
                logger.warning(f"Error during power down 1V8: {e}")

        # Log final result
        if test_passed:
            msg = f"TEST PASSED in {elapsed:.2f} sec"
            logger.info(msg)
            print(msg) # For console visibility
        else:
            msg = f"TEST FAILED in {elapsed:.2f} sec due to {failure_reason}"
            logger.error(msg)
            print(msg) # For console visibility

        return test_passed

if __name__ == "__main__":
    run_test()