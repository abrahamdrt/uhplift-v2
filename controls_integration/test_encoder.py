import spidev
import time
import sys

def main():
    print("=== New Encoder Board SPI Diagnostic ===")
    print("Bus: SPI0 | CS: CE0 (GPIO 8)")
    
    # Initialize SPI0 on CE0
    try:
        spi = spidev.SpiDev()
        spi.open(0, 0)
        spi.max_speed_hz = 50_000
        spi.mode = 0b00  # Mode 0 is standard for the LS7366R
    except FileNotFoundError:
        print("\n[ERROR] SPI0 is not enabled. Run 'sudo raspi-config' to enable it.")
        sys.exit(1)

    # LS7366R Instruction Opcodes
    WRITE_MDR0 = 0x88
    READ_MDR0  = 0x48
    
    # We will write 0x03 to MDR0 (this sets it to standard 4-byte counter mode)
    test_value = 0x03

    print(f"\nWriting 0x{test_value:02X} to MDR0 register...")
    
    try:
        # Send the Write Opcode followed by the data
        spi.xfer2([WRITE_MDR0, test_value])
        
        # Give the chip a tiny microsecond fraction to process
        time.sleep(0.01)
        
        print("Reading back MDR0 register...")
        # Send the Read Opcode, followed by a dummy byte (0x00) to clock the data back
        response = spi.xfer2([READ_MDR0, 0x00])
        
        # The counter responds during the second byte transfer
        readback_value = response[1]
        
        print(f"Received: 0x{readback_value:02X}")
        
        if readback_value == test_value:
            print("\n[DIAGNOSIS]: SUCCESS! The Pi and the counter are talking perfectly.")
        elif readback_value == 0x00 or readback_value == 0xFF:
            print("\n[DIAGNOSIS]: FAILED.")
            print("Received dead air. Check your 5V power, ground, and MISO/MOSI wiring.")
        else:
            print("\n[DIAGNOSIS]: GARBAGE DATA.")
            print("Received a mismatched number. Check for loose jumper wires or clock noise.")
            
    except Exception as e:
        print(f"\n[ERROR] {e}")
    finally:
        spi.close()

if __name__ == "__main__":
    main()