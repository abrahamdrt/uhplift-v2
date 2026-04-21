import spidev

spi = spidev.SpiDev()
spi.open(1, 0)         # SPI1, CE0
spi.mode = 0
spi.max_speed_hz = 1000000

# Send a test pattern
msg = [0xAA, 0x55, 0x0F, 0x8F]
resp = spi.xfer2(msg)

print(f"Sent: {[hex(x) for x in msg]}")
print(f"Got:  {[hex(x) for x in resp]}")

if msg == resp:
    print("Loopback: PASS! The Pi's hardware and pins are perfect.")
else:
    print("Loopback: FAIL! You are plugged into the wrong pins.")