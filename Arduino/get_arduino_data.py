from smbus import SMBus
import time

bus = SMBus(3) # Raspberry Pi revision 2
address = 0x64

while True:
    data = bus.read_i2c_block_data(address,0)
    print data[0:4]
    time.sleep(0.1)
