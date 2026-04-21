import spidev
import time

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 1) # Bus 0, Device 1 (GPIO 7 / CE1)

# Configuration based on LSM6DS3 Datasheet
spi.max_speed_hz = 1000000 # Start slow at 1MHz for troubleshooting
spi.mode = 0b11            # SPI Mode 3 (CPOL=1, CPHA=1) [cite: 335]

def read_who_am_i():
    # WHO_AM_I address is 0x0F [cite: 351]
    # For a READ, bit 0 must be 1 -> 0x0F | 0x80 = 0x8F [cite: 346, 351]
    address = 0x8F 
    
    # Send 0x8F and a dummy byte (0x00) to clock out the response [cite: 610]
    response = spi.xfer2([address, 0x00])
    
    # The first byte returned is garbage; the second is the data [cite: 588]
    return response[1]

try:
    print("Starting IMU Communication Test...")
    while True:
        device_id = read_who_am_i()
        
        if device_id == 0x69: # [cite: 361, 371]
            print(f"SUCCESS: Found LSM6DS3! ID: {hex(device_id)}")
        elif device_id == 0x00 or device_id == 0xFF:
            print(f"FAILURE: No connection. Received {hex(device_id)}. Check wiring/soldering.")
        else:
            print(f"UNEXPECTED: Received {hex(device_id)}. Expected 0x69.")
            
        time.sleep(1)

except KeyboardInterrupt:
    spi.close()
    print("\nTest stopped by user.")