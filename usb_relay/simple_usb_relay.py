import serial
import time
import codecs

# parameters
port = 'COM5'
baudrate = 9600

LED_delay = 0.5

s = serial.Serial(port,baudrate,stopbits=1)
s.close()

while(True):
    
    # TO close relay (on)
    s.open()
    on = 'A00101A2'
    on_hex = codecs.decode(on, 'hex')
    s.write(on_hex)
    s.close()
    
    # Give time delay
    time.sleep(LED_delay)
   
    # To open relay (off)
    s.open()
    off = 'A00100A1'
    off_hex = codecs.decode(off, 'hex')
    s.write(off_hex)
    s.close()
    
    # Give time delay
    time.sleep(LED_delay)
    
