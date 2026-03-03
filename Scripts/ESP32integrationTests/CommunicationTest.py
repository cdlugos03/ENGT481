import numpy as np
import sympy as sp
import control as ctrl
import serial
import time
import struct

#angle corrections
downVal = 11060
correction = downVal - 8192

#initialize serial
# cereal = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.003) #initiate communication with the ESP32
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=0.003) #initiate communication with the arduino
time.sleep(3) #since code is restarted gives time for arduino
arduino.reset_input_buffer() #clears any old log before reading data
print("\nSerial started\n") #confirms this connection
line = 0x00000000

#get first measurement

#send blank control value
pulses = -65535
#arrange as a 32 bit signed integer for transmission
sendPulses = struct.pack('<i', int(pulses))
arduino.write(sendPulses) #send top value to Arduino

f = 0
fDir = 1



try:
    while True:        
        
        #Frequency sweep
        time.sleep(0.002) # wait 5ms
        
        f = f + fDir #increment f
        
        if f > 6000: #toggle direction
            fDir = -1
        elif f < -6000:
            fDir = 1

        
#         print("pos:", x)
#         print("angle:", angleRaw)

#         currentTime = int(time.time() * 1000)
# 
#         if len(line) != 4:
#             print("values dropped")
#             print(currentTime)
#         else:
#             message = struct.unpack('>i', line)[0]
#         
#         print(message)
#         print(currentTime)

        #convert frequency to timer top value
        if f > 2 or f < -2:
            conversion = (0.5/f)/(1.0/250000.0)
            pulses = conversion
        elif f >= 0:
            pulses = 65535
        elif f < 0:
            pulses = -65535
        
        print(round(pulses))
        
        #arrange as a 32 bit signed integer for transmission
        sendPulses = struct.pack('<i', int(pulses))
        arduino.write(sendPulses) #send top value to Arduino
#         cereal.write('\n') #indent

        
        #wait for encoder values
        line = bytearray()
        while len(line) < 2: #read until line is filled
            line.extend(arduino.read(2 - len(line))) #reads response from Arduino
        #convert stream to independent variables
        if len(line) != 2:
            print("values dropped")
            print(list(line))
        else:
            positionRaw = struct.unpack('>h', line)
            #convert positionRaw to meters and angleRaw to radians
#             print(round(theta, 3))
#             print(round(correction))
            print(positionRaw)
        arduino.reset_input_buffer()
        

#this section kills the program
except KeyboardInterrupt: # to end program use ctrl c
    print("\nControl loop stopped\n")
    arduino.write(0xFFFFFFFF) #send stop command to Arduino
    #cereal.write('\n') #indent
    time.sleep(0.2) #wait 200ms
    arduino.close() #ends connection between devices