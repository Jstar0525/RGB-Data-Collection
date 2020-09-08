USB RELAY
=========

This code is python and can run usb relay.

Product detail (1 Channel USB relay)
------------------------------------

<img src="../docs/relay/usb_relay.jpg" width="40%" height="30%"></img>

This one can buy [here (1channel usb relay)](http://vctec.co.kr/product/1%EC%B1%84%EB%84%90-usb-%EB%A6%B4%EB%A0%88%EC%9D%B4-10a-1-channel-usb-relay-10a/14806/#none)

### 1. Outline
* This product is 1 channel relay.
* You can control relay through usb.
* Up to 10A switching is possible.
* The USB driver chip uses the CH340 chip, so it can be recognized as a serial port by the PC.

### 2. Characteristics
* LED indicator: Onboard power LED and relay status LED
* Relays: Onboard 5V, 10A / 250VAC, 10A / 30VDC relays, long relay life, can be continuous pull 10 million
* Protection function: with overcurrent protection and relay diode freewheeling protection.

### 3. Protocol

Data (1) : starting identity (the default is 0xA0)   
Data (2) : switch address code (default is 0x01, identifying the first channel switch)   
Data (3) : Operating Data (0x00 to "Off", 0x01 "On")   
Data (4) : checksum   

##### Example:
Open USB switch: A0 01 01 A2   
Close USB switch: A0 01 00 A1

##### Operation:

Step1: the USB relay module into the computer, install CH340 USB to serial chip driver

Step 2: Open the STC-ISP, SSCOM32 serial debugging software, select the baud rate of 9600,    
in hexadecimal (hex) form to send A0 01 01 A2 to open the relay; hex (hex) form Send A0 01 00 A1 to turn off the relay,    
you can choose to send manually or automatically. 

The following: SSCOM332 serial debugging software for example:   
* Select the baud rate 9600, check the "HEX send", enter the command A0 01 01 A2 or A0 01 00 A1, point "send", you can turn on or off the relay   
* Select the baud rate 9600, click on the SSCOM32 interface in the "expansion", enter the command a0 01 01 A2 and A0 01 00 A1,    
check the "HEX send", enter the automatic cycle to send the interval, check the "automatic loop" The relay can be automatically turned on and off

Usage
-----

* if you want to run **simple_usb_relay.py**, you have to check 'Port' in 'Device Manager'(on Windows10)   
* **auto_usb_relay.py** automatically check port. so, you can run program directly.
