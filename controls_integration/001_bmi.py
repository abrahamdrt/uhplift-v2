import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 1) # SPI0, CE1
spi.max_speed_hz = 500000
spi.mode = 0    # BMI160 is more stable in Mode 0 on Pi

def check_id():
    # Read address 0x00 (BMI160 Chip ID)
    # 0x80 is the Read Bit
    resp = spi.xfer2([0x80, 0x00])
    print(f"Chip ID Received: {hex(resp[1])}")
    if resp[1] == 0xd1:
        print("CONFIRMED: This is a Bosch BMI160.")
    elif resp[1] == 0xff:
        print("STATUS: Bus Floating (0xFF). Check if CS is actually connected to GPIO 7.")
    else:
        print("STATUS: Unknown chip or noise.")

if __name__ == "__main__":
    check_id()
    spi.close()