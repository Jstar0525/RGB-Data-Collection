import time
import codecs
import warnings
import serial
import serial.tools.list_ports

# Parameters
baudrate = 9600
LED_delay = 0.5

# Find CH340 port
CH340_port = [
    p.device
    for p in serial.tools.list_ports.comports()
    if 'CH340' in p.description
    ]

if not CH340_port:
    raise IOError('No CH340 found')
    
if len(CH340_port) > 1:
    warnings.warn('Multiple CH340 found - using the first')
    
# Serial setting
s = serial.Serial(CH340_port[0],baudrate,stopbits=1)
s.close()
    
    
while(True):
    
    # To open relay (off)
    s.open()
    off = 'A00100A1'
    off_hex = codecs.decode(off, 'hex')
    s.write(off_hex)
    s.close()
    print('LED OFF')
    
    # Give time delay
    time.sleep(LED_delay)
    
    # TO close relay (on)
    s.open()
    on = 'A00101A2'
    on_hex = codecs.decode(on, 'hex')
    s.write(on_hex)
    s.close()
    print('LED ON')
    
    # Give time delay
    time.sleep(LED_delay)
    
