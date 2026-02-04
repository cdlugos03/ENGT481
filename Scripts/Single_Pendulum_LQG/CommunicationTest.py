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
cereal = serial.Serial('/dev/ttyACM0', 115200, timeout=0.003) #initiates connection, will restart arduino code
time.sleep(3) #since code is restarted gives time for arduino
cereal.reset_input_buffer() #clears any old log before reading data
print("\nSerial started\n") #confirms this connection
line = 0x00000000

#send blank control value to start transmission
pulses = -65535
#arrange as a 32 bit signed integer for transmission
sendPulses = struct.pack('<i', int(pulses))
cereal.write(sendPulses) #send top value to Arduino

try:
    while True:
        #wait for arduino values
        while cereal.in_waiting <= 3: #infinitely waits until arduino sends serial value
            #time.sleep(0.000001) #small delay (1us) to keep pi from overloading
            pass
        line = cereal.readline().rstrip() #reads response from Arduino
        #convert stream to independent variables
        if len(line) != 4:
            print("values dropped")
        else:
            positionRaw, angleRaw = struct.unpack('>hh', line)
            angleRaw2 = angleRaw - correction
            #convert positionRaw to meters and angleRaw to radians
            x = (positionRaw*0.638175)/6400 #based on distance measurement (m) for 6400 steps
            theta = ((angleRaw2*2*np.pi)/16383) #THIS ONLY WORKS FOR 14 BIT
        
        
#         print("pos:", x)
        print("angle:", angleRaw)

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

# 
#         f = -100
#         #convert frequency to timer top value
#         if f > 2 or f < -2:
#             conversion = (0.5/f)/(1.0/250000.0)
#             pulses = conversion
#         elif f >= 0:
#             pulses = 65535
#         elif f < 0:
#             pulses = -65535
            
        pulses = -65535
        
        #arrange as a 32 bit signed integer for transmission
        sendPulses = struct.pack('<i', int(pulses))
        cereal.write(sendPulses) #send top value to Arduino
#         cereal.write('\n') #indent
        

#this section kills the program
except KeyboardInterrupt: # to end program use ctrl c
    print("\nControl loop stopped\n")
    cereal.write(0xFFFFFFFF) #send stop command to Arduino
    #cereal.write('\n') #indent
    cereal.close() #ends connection between devices