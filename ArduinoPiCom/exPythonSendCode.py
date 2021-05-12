from datetime import datetime
from datetime import timedelta
from smbus import SMBus
import struct

start_time = datetime.now()

def millis():
    dt = datetime.now()-start_time
    ms = (dt.days*24*60*60 + dt.seconds)*1000+dt.microseconds / 1000.0
    return ms

#start slave i2c
bus = SMBus(1) #the i2c of this RPi starts with 1
arduinoAddress = 12

#execution interval
interval = 150

temperatura = 10.2
vazao = 5.3
command = 20
teste = 30

if __name__ == '__main__':
    prevmillis = millis()

    while True:
        currentmillis = millis()
        if(currentmillis - prevmillis > interval):

            #write

            bytescommand = struct.pack('=2fbb',temperatura,vazao,command,teste) #to avoid adjustment
            bus.write_block_data(arduinoAddress,1,list(bytescommand))
            print(list(bytescommand))

            #request

            # block = bus.read_i2c_block_data(arduinoAddress,2,27)
            # output = struct.unpack('6f3b',bytes(block))
            # print(output)
            # print(datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3])
            prevmillis = currentmillis
