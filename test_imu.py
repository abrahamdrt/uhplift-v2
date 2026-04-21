#!/usr/bin/env python3
import spidev
import time

def test_lsm6ds3():
    print("--- LSM6DS3 SPI1 Diagnostic ---")
    try:
        # Initialize SPI1, CE0 (GPIO 18)
        spi = spidev.SpiDev()
        spi.open(1, 0) 
        spi.mode = 0b00  # LSM6DS3 uses Mode 0
        spi.max_speed_hz = 1000000
        
        # WHO_AM_I Register is 0x0F. 
        # For a read operation, the MSB must be set to 1.
        # 0x0F | 0x80 = 0x8F
        reg_to_read = 0x0F | 0x80
        
        # Send register address, read back 1 byte of data (send dummy 0x00)
        resp = spi.xfer2([reg_to_read, 0x00])
        
        who_am_i = resp[1]
        print(f"WHO_AM_I Register (0x0F) returned: 0x{who_am_i:02X}")
        
        if who_am_i == 0x69:
            print("SUCCESS: LSM6DS3 is communicating perfectly!")
        else:
            print("ERROR: Unexpect WHO_AM_I")
            
        spi.close()
        
    except FileNotFoundError:
        print("ERROR: /dev/spidev1.0 not found. Did you add 'dtoverlay=spi1-1cs' to config.txt?")
    except Exception as e:
        print(f"ERROR: {e}")

if __name__ == "__main__":
    test_lsm6ds3()