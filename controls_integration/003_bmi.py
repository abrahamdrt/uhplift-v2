import spidev
import time

# Initialize SPI1
spi = spidev.SpiDev()
spi.open(1, 0)  # SPI bus 1, device 0 (CS0 on GPIO 18)
spi.max_speed_hz = 5000000 # 5MHz
spi.mode = 0    # Must be Mode 0 for SPI1

def read_chip_id():
    # BMI160: Bit 7 high for read. CHIP_ID is register 0x00.
    # So we send 0x80 (0x00 | 0x80) and a dummy byte.
    msg = [0x80, 0x00] 
    reply = spi.xfer2(msg)
    return reply[1]

try:
    print("Toggling CS to force SPI mode...")
    # Just reading once to wake up SPI interface
    read_chip_id() 
    time.sleep(0.1)
    
    chip_id = read_chip_id()
    if chip_id == 0xD1:
        print(f"Success! BMI160 found. Chip ID: {hex(chip_id)}")
    else:
        print(f"Communication failed. Expected 0xD1, got: {hex(chip_id)}")
        print("Check your wiring and VIN power.")

finally:
    spi.close()