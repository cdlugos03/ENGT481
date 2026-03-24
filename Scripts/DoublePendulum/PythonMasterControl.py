import numpy as np
import sympy as sp
import control as ctrl
import serial
import time
import struct
import sys
import os
import re

print("program started")

# Define the file path
script_dir = os.path.dirname(os.path.abspath(__file__))
file_path = os.path.join(script_dir, "control_matrices.txt")

def load_control_data(filename):
    with open(filename, 'r') as f:
        content = f.read()

    mapping = {
        'A': r'--- Matrix A \(Linearized\) ---\s*(.*?)(?=---|\Z)',
        'B': r'--- Matrix B ---\s*(.*?)(?=---|\Z)',
        'C': r'--- Matrix C ---\s*(.*?)(?=---|\Z)',
        'D': r'--- Matrix D ---\s*(.*?)(?=---|\Z)',
        'Kd': r'--- LQR Gain \(Kd\) ---\s*(.*?)(?=---|\Z)',
        'Ld': r'--- Kalman Gain \(Ld\) ---\s*(.*?)(?=---|\Z)'
    }

    results = {}

    for var_name, pattern in mapping.items():
        match = re.search(pattern, content, re.DOTALL)
        if match:
            raw_text = match.group(1)
            
            clean_text = re.sub(r'\*', '', raw_text) 
            
            # Remove brackets and replace newlines with spaces to keep numbers separate
            clean_text = clean_text.replace('[', '').replace(']', '').replace('\n', ' ')
            
            # Convert string numbers to float list
            # .split() handles multiple spaces automatically
            data_list = [float(x) for x in clean_text.split() if x]
            
            # Reconstruct the shapes
            if var_name == 'A':
                results[var_name] = np.array(data_list).reshape(6, 6)
            elif var_name == 'B':
                results[var_name] = np.array(data_list).reshape(6, 1)
            elif var_name == 'C':
                results[var_name] = np.array(data_list).reshape(3, 6)
            elif var_name == 'D':
                results[var_name] = np.array(data_list).reshape(3, 1)
            elif var_name == 'Kd':
                results[var_name] = np.array(data_list).reshape(1, 6)
            elif var_name == 'Ld':
                results[var_name] = np.array(data_list).reshape(6, 3)

    return results

# Usage
data = load_control_data(file_path)

# --- Variable data assignment --- #
A = data['A'] 
B = data['B'] 
C = data['C']
D = data['D']
Kd = data['Kd'] 
Ld = data['Ld']

print("Successfully loaded Matrix A with shape:", A.shape)
# print(A)

#-----CONTROL CODE-------#
line = 0x00000000
positionRaw = 0x0000
angleRaw = 0x0000
x = 0 #cart position
theta = 0 #pendulum angle
xDiv = 0 #cart velocity
f = 0
conversion = 0
pulses = 0
#thetaDiv = 0
u = np.array([[0.0]]) #control force
Ymeas = np.array([[0], [0]]) #measured states (no thetaDiv)
Yest = np.array([[0], [0], [0], [0]]) #estimated states
Ylast = np.array([[0], [0], [0], [0]]) #previous state storage
YfinalEst = np.array([[0], [0], [0], [0]]) #previous state storage

#angle corrections
downVal = 10675 - 8192 #for pendulum 1
if downVal > 8192:
    correction = downVal - 8192
elif downVal == 8192:
    correction = 0
else:
    correction = downVal + 8192

correction2 = 8250 #for pendulum 2 PLACEHOLDER VALUE, MAKE SURE TO CHANGE

#initialize serial
esp32 = serial.Serial('/dev/ttyUSB0', 921600, timeout=0.003) #initiate communication with the ESP32
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=0.003) #initiate communication with the arduino
time.sleep(3) #since code is restarted gives time for arduino
arduino.reset_input_buffer() #clears any old log before reading data
print("\nSerial started\n") #confirms this connection
line = 0x00000000

time.sleep(0.1)


#get first angle measurements
#Read encoder values
theta1, theta2 = angleRead(correction, correction2)


#send low frequency control value to trigger arduino response
pulses = -65535
#arrange as a 32 bit signed integer for transmission
sendPulses = struct.pack('<i', int(pulses))
arduino.write(sendPulses) #send top value to Arduino

#get position from arduino
x = positionRead()

#store initial measurements in a matrix
Ylast = np.array([[theta1], [0], [theta2], [0], [x], [0]])

loop = 0 #loop counting variable

try:
    while True:
        if loop < 250: #only count until loop 251
            loop = loop + 1
        
        #calculate predicted states (Kalman filter part 1)
        Yest = ssDisc.A @ Ylast + ssDisc.B @ u

        theta1, theta2 = angleRead(round(correction), correction2)#read angle
#        theta1 = theta1 - np.clip(x/20, a_min=-0.05, a_max=0.05) #correct angle towards center
        
        #load measurements into matrix (using position from last loop)
        Ymeas = np.array([[theta1], [theta2], [x]])
        
        #Factor in measured states(Kalman filter part 2) (@ for matrix multiplication)
        YfinalEst = Yest + Ld @ (Ymeas - ssDisc.C @ Yest)
        Ylast = YfinalEst.copy() #store values for next loop
        
        #calculate control force
        u = -Kd @ YfinalEst
        if loop < 200: #ramp force up to full
            u = u*(loop/200.0)
        #MAY NEED TO IMPLEMENT PID CORRECTIONS FOR DRIFT
        
        #convert force to velocity and frequency (u.item takes value from 1x1 array)
        aCart = u.item() / mcartVal #calculate target cart acceleration
        xDivLast = xDiv #store last speed
        xDiv = xDivLast + aCart * Ts #calulate target cart speed
        f = -(xDiv * 6400) / 0.638175 #conversion based on measured distance per pulse
        
        #convert frequency to timer top value
        if f > 2 or f < -2:
            conversion = (0.5/f)/(1.0/250000.0)
            pulses = conversion
        elif f >= 0:
            pulses = 65535
        elif f < 0:
            pulses = -65535
            
        #send timer value to arduino
        arduino.reset_output_buffer()
        #arrange as a 32 bit signed integer for transmission
        sendPulses = struct.pack('<i', int(pulses))
        arduino.write(sendPulses) #send top value to Arduino
        
        x = positionRead() #read position from arduino
        correction = correction + x/50 #correct correction value slightly
        
        
        

#this section kills the program
except KeyboardInterrupt: # to end program use ctrl c
    print("\nControl loop stopped\n")
    arduino.write(0xFFFFFFFF) #send stop command to Arduino
    time.sleep(0.05) #wait 50ms
    arduino.close() #ends connection between devices

    sys.exit()
