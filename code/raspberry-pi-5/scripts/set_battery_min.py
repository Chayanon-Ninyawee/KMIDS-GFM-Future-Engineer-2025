import time

import smbus2

DEVICE_BUS = 1
DEVICE_ADDR = 0x17  # Default I2C address for EP-0136


def write_register_16(reg_low, value_mv):
    """Write 16-bit value (in mV) to two consecutive registers (low, high)."""
    bus = smbus2.SMBus(DEVICE_BUS)
    low = value_mv & 0xFF
    high = (value_mv >> 8) & 0xFF
    bus.write_byte_data(DEVICE_ADDR, reg_low, low)
    bus.write_byte_data(DEVICE_ADDR, reg_low + 1, high)
    bus.close()


def read_register_16(reg_low):
    """Read 16-bit value from two consecutive registers (low byte first)."""
    bus = smbus2.SMBus(DEVICE_BUS)
    low = bus.read_byte_data(DEVICE_ADDR, reg_low)
    high = bus.read_byte_data(DEVICE_ADDR, reg_low + 1)
    return (high << 8) | low


if __name__ == "__main__":
    print(f"Battery Full Voltage: {read_register_16(0x0D)} mV")
    print(f"Battery Empty Voltage: {read_register_16(0x0F)} mV")
    print(f"Battery Protection Voltage: {read_register_16(0x11)} mV")

    time.sleep(0.2)
    write_register_16(0x0D, 4200)  # Full Voltage
    time.sleep(0.2)
    write_register_16(0x0F, 3200)  # Empty Voltage
    time.sleep(0.2)
    write_register_16(0x11, 3000)  # Protection Voltage
    time.sleep(0.2)

    print(f"\nBattery Full Voltage: {read_register_16(0x0D)} mV")
    print(f"Battery Empty Voltage: {read_register_16(0x0F)} mV")
    print(f"Battery Protection Voltage: {read_register_16(0x11)} mV")
