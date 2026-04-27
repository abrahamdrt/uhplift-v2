import spidev
import pigpio
import time
import sys

CS_PIN = 18

# 1. Connect to the pigpio daemon for precise pin control
pi = pigpio.pi()
if not pi.connected:
    print("❌ pigpio daemon not running. Run 'sudo pigpiod' first.")
    sys.exit()

# Ensure CS starts HIGH
pi.set_mode(CS_PIN, pigpio.OUTPUT)
pi.write(CS_PIN, 1)

# 2. Initialize SPI
spi = spidev.SpiDev()
spi.open(1, 0)
spi.max_speed_hz = 500000
spi.mode = 0
spi.no_cs = True  # CRITICAL: Tells the kernel to ignore hardware CS

def read_reg(reg):
    # --- MANUAL CHIP SELECT ---
    pi.write(CS_PIN, 0)   # 1. Pull CS Low manually
    time.sleep(0.0001)    # 2. Wait 100 microseconds (gives IMU time to wake)
    
    msg = [reg | 0x80, 0x00]
    reply = spi.xfer2(msg) # 3. Send the data clock
    
    time.sleep(0.0001)    # 4. Wait 100us for transaction to finish settling
    pi.write(CS_PIN, 1)   # 5. Push CS High manually
    # --------------------------
    return reply[1]

try:
    print("--- BMI160 Manual CS Override ---")
    print("1. Forcing manual CS toggles...")
    
    # Send dummy reads to trigger the I2C -> SPI switch
    for _ in range(5):
        read_reg(0x7F)
        time.sleep(0.01)

    print("2. Reading CHIP_ID (0x00)...")
    chip_id = read_reg(0x00)
    
    if chip_id == 0xD1:
        print("✅ SUCCESS! Chip ID 0xD1 found!")
        print("The chip is perfectly healthy. The Linux SPI1 driver was just mangling the CS timing.")
    else:
        print(f"❌ Failed. Returned: 0x{chip_id:02X}")
        print("If it's still 0x00 or 0xFF, the hardware is truly unresponsive.")

finally:
    spi.close()
    pi.write(CS_PIN, 1)
    pi.stop()