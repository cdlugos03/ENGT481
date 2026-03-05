import numpy as np
import sympy as sp
import control as ctrl
import serial
import time
import struct
import sys

def angleRead(cor):
    #Read encoder values
    line = bytearray()
    while len(line) < 6: #read until line is filled
        line.extend(esp32.read(6 - len(line))) #reads response from Arduino
    #convert stream to independent variables
    if len(line) != 6:
        print("values dropped")
        print(list(line))
        return 0
    else:
        angle1, angle2, angle3 = struct.unpack('>HHH', line)
        #convert positionRaw to meters and angleRaw to radians
        angle1 = angle1 - cor
        theta = ((angle1*2*np.pi)/16383) #THIS ONLY WORKS FOR 14 BIT
        #             print(round(theta, 3))
        #             print(round(correction))
        print(theta, angle1)
        return theta
    esp32.reset_input_buffer()

def positionRead():
    #wait for position value
    line = bytearray()
    while len(line) < 2: #read until line is filled
        line.extend(arduino.read(2 - len(line))) #reads response from Arduino
    #convert stream to independent variables
    if len(line) != 2:
        print("values dropped")
        print(list(line))
        return 0
    else:
        positionRaw = struct.unpack('>h', line)[0]
        #convert positionRaw to meters and angleRaw to radians
        x = (positionRaw*0.638175)/6400 #based on distance measurement (m) for 6400 steps
#         print(positionRaw)
        return x
    arduino.reset_input_buffer()


#define symbols and symbol properties
t,g,l,m1,mcart,T_drag,B_cart_drag = sp.symbols('t g l m1 mcart T_drag B_cart_drag', positive = True)
theta = sp.Function('theta')(t) #define theta as a function of t
x = sp.Function('x')(t) #define x as a function of t
I,H,V,F,F_drag = sp.symbols('I H V F F_drag', real = True)

#Sample Period
Ts = 1/200

#0.5N rolling friction

#actual system values
length = 0.25
mweight = 0.030 #weight of pendulum weight
mrod = 0.072 #weight of pendulum arm
lval = 0.16#(mweight*length+mrod*(length/2))/(mweight+mrod) #in meters (center of mass)  #FIX THIS YOU IDIOT
F_dragVal = 0.05 #pendulum drag force
Ival = (mweight + (mrod/3))*lval**2 #rotational inertia
mcartVal = 0.85 #in kg
B_cart_dragVal = 0.5 #drag coefficient
gval = 9.81 #gravitational constant
m1val = mweight + mrod #in kg
#substitutions
vals = {m1:m1val,mcart:mcartVal,l:lval,g:gval,I:Ival,F_drag:F_dragVal,B_cart_drag:B_cart_dragVal}

#define equations
CartH = sp.Eq(F - H - B_cart_drag * sp.Derivative(x,t), mcart * sp.Derivative(x,t,2))
HorizontalForce = sp.Eq(H - F_drag, m1 * sp.Derivative(x-l*sp.sin(theta),t,2))

Hsolve = sp.solve(HorizontalForce, H)[0] #solve for H, taking only the first(only in this case) solution [0]
mainEq1 = CartH.subs(H, Hsolve) #substitute in Hsolve for H

Normal_to_bar = sp.Eq(F_drag*sp.cos(theta) - V*sp.sin(theta) - H*sp.cos(theta) + m1*g*sp.sin(theta), m1*(l*sp.Derivative(theta,t,2) - sp.Derivative(x,t,2)*sp.cos(theta)))
Torque = sp.Eq(V*l*sp.sin(theta) + H*l*sp.cos(theta), I*sp.Derivative(theta,t,2))

Normal_to_bar = Normal_to_bar.subs(H, Hsolve) #substitute in Hsolve for H
Torque = Torque.subs(H, Hsolve)

Vsolve = sp.solve(Torque, V)[0] #solve for V
mainEq2 = sp.simplify(Normal_to_bar.subs(V, Vsolve))


#move everything to one side of the equation (set equal to 0)
Eq1 = mainEq1.lhs - mainEq1.rhs
Eq2 = mainEq2.lhs - mainEq2.rhs
#set up symbols for x2div and theta2div
x2div,theta2div = sp.symbols('x2div theta2div', real = True)
#substitute in new symbols in place of accelerations
Eq1 = Eq1.subs({sp.Derivative(x,t,2):x2div, sp.Derivative(theta,t,2):theta2div})
Eq2 = Eq2.subs({sp.Derivative(x,t,2):x2div, sp.Derivative(theta,t,2):theta2div})

#solve equations for accelerations
Subs = sp.solve([Eq1, Eq2], [x2div, theta2div], simplify = True)

#substitute in symbols for each state
Y1,Y2,Y3,Y4 = sp.symbols('Y1 Y2 Y3 Y4')
StateSubs = {theta:Y1, sp.Derivative(theta,t):Y2, x:Y3, sp.Derivative(x,t):Y4}
F1 = Subs[theta2div].subs(StateSubs)
F2 = Subs[x2div].subs(StateSubs)

#combine into a matrix (non-linear state space model), and create state matrix (Y)
NonLinMod = sp.Matrix([Y2, F1, Y4, F2])
Y = sp.Matrix([Y1, Y2, Y3, Y4])

#linearize model
A = NonLinMod.jacobian(Y)
B = NonLinMod.diff(F)
C = sp.Matrix([ #manual input
    [1, 0, 0, 0],
    [0, 0, 1, 0],])
D = sp.Matrix([0, 0]) #manual input

#Substitute in values
Equilibrium = {Y1:0,Y2:0,Y3:0,Y4:0} #all states are 0 at equilibrium
A = A.subs(vals).subs(Equilibrium)
B = B.subs(vals).subs(Equilibrium)

#convert to a ss system, and discretize
ssCont = ctrl.ss(A, B, C, D)
ssDisc = ctrl.c2d(ssCont, Ts)




#-----TUNING VALUES-----#
#calculate lqr and kalman filter values
#K, S, E = ctrl.lqr(A, B, sp.diag(10,2,1,1), 0.5)
Kd, Sd, Ed = ctrl.dlqr(ssDisc, sp.diag(10,1,9,0.5), 1)
#QN and RN are multiplied/divided by Ts to discretize them. MATLAB does this internally
Ld, Pd, Edkalm = ctrl.dlqe(ssDisc.A, sp.diag(1,1,1,1), ssDisc.C, Ts*sp.diag(0.2,0.001,0.2,0.001), (0.01/Ts)*sp.diag(1,1))







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
downVal = 11078 - 8192
if downVal > 8192:
    correction = downVal - 8192
elif downVal == 8192:
    correction = 0
else:
    correction = downVal + 8192

#initialize serial
esp32 = serial.Serial('/dev/ttyUSB0', 921600, timeout=0.003) #initiate communication with the ESP32
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=0.003) #initiate communication with the arduino
time.sleep(3) #since code is restarted gives time for arduino
arduino.reset_input_buffer() #clears any old log before reading data
print("\nSerial started\n") #confirms this connection
line = 0x00000000

#get first angle measurements
#Read encoder values
theta1 = angleRead(correction)


#send low frequency control value to trigger arduino response
pulses = -65535
#arrange as a 32 bit signed integer for transmission
sendPulses = struct.pack('<i', int(pulses))
arduino.write(sendPulses) #send top value to Arduino

#get position from arduino
x = positionRead()

Ylast = np.array([[theta1], [0], [x], [0]])

loop = 0

try:
    while True:
        if loop < 250:
            loop = loop + 1
        #calculate predicted states (Kalman filter part 1)
        Yest = ssDisc.A @ Ylast + ssDisc.B @ u 


        theta1 = angleRead(round(correction))#read angle
        theta1 = theta1 - np.clip(x/20, a_min=-0.05, a_max=0.05) #correct angle towards center
        
        #load measurements into matrix (using position from last loop)
        Ymeas = np.array([[theta1], [x]])
        
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
            
        arduino.reset_output_buffer()
        
        #arrange as a 32 bit signed integer for transmission
        sendPulses = struct.pack('<i', int(pulses))
        arduino.write(sendPulses) #send top value to Arduino
        
        x = positionRead() #read position from arduino
        correction = correction + x/50 #correct correction value slightly
        
        
        

#this section kills the program
except KeyboardInterrupt: # to end program use ctrl c
    print("\nControl loop stopped\n")
    cereal.write(0xFFFFFFFF) #send stop command to Arduino
    cereal.close() #ends connection between devices
    sys.exit()