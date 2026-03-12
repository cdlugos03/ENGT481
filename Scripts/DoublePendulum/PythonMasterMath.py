import numpy as np
import sympy as sp
import control as ctrl
import serial
import time
import struct
import sys
import os


print("program started")


def angleRead(cor, cor2):
    #Read encoder values
    line1 = bytearray()
    while len(line1) < 6: #read until line is filled
        line1.extend(esp32.read(6 - len(line1))) #reads response from Arduino
    #convert stream to independent variables
    if len(line1) != 6:
        print("values dropped")
        print(list(line1))
        return 0
    else:
        angle1, angle2, angle3 = struct.unpack('>HHH', line1)
        #convert positionRaw to meters and angleRaw to radians
        angle1 = angle1 - cor
        thetaRead = ((angle1*2*np.pi)/16383) #THIS ONLY WORKS FOR 14 BIT
        angle2 = angle2 - cor2
        theta2Read = ((angle2*2*np.pi)/16383) #THIS ONLY WORKS FOR 14 BIT
        #             print(round(theta, 3))
        #             print(round(correction))
        print(thetaRead, angle1)
        esp32.reset_input_buffer()
        return thetaRead, theta2Read
 
def positionRead():
    #wait for position value
    line2 = bytearray()
    while len(line2) < 2: #read until line is filled
        line2.extend(arduino.read(2 - len(line2))) #reads response from Arduino
    #convert stream to independent variables
    if len(line2) != 2:
        print("values dropped")
        print(list(line2))
        arduino.reset_input_buffer()
        return 0
    else:
        positionRaw = struct.unpack('>h', line2)[0]
        #convert positionRaw to meters and angleRaw to radians
        x1 = (positionRaw*0.638175)/6400 #based on distance measurement (m) for 6400 steps
#         print(positionRaw)
        arduino.reset_input_buffer()
        return x1

#define symbols and symbol properties
t,g,l,l2,m1,m2,mcart,B_cart_drag = sp.symbols('t g l l2 m1 m2 mcart B_cart_drag', positive = True)
theta = sp.Function('theta')(t) #define theta as a function of t
theta2 = sp.Function('theta2')(t)
x = sp.Function('x')(t) #define x as a function of t
I,I2,F,T_drag = sp.symbols('I I2 F T_drag', real = True)

#Sample Period
Ts = 1/200

ArrayPrec = 5 #how many decimals are sent to the Text file

#0.5N rolling friction

#actual system values
length = 0.25
mweight = 0.03 #weight of pendulum weight
mrod = 0.072 #weight of pendulum arm
lval = 0.185#(mweight*length+mrod*(length/2))/(mweight+mrod) #in meters (center of mass)  #FIX THIS YOU IDIOT
Ival = 0.0006772 #((mweight + (mrod/3))*lval**2) #rotational inertia
m1val = 0.151#mweight + mrod #in kg

# length2 = 0.25 #probably not needed for anything
length2 = 0.2
mrod2 = 0.051
mweight2 = 0.022
mtotal2 = mrod2 + mweight2
lval2 = 0.12 #length to center of mass
Ival2 = 0.0003667 #((mweight2 + (mrod2/3))*length2**2) #rotational inertia

mcartVal = 0.969 #in kg
B_cart_dragVal = 0.5 #drag coefficient
gval = 9.81 #gravitational constant
T_dragVal = 0.01 #rotational drag force

#substitutions
vals = {m1:m1val,m2:mtotal2,mcart:mcartVal,l:lval,l2:lval2,g:gval,I:Ival,I2:Ival2,T_drag:T_dragVal,B_cart_drag:B_cart_dragVal}

#define equations
T =	(
    (1/2)*(mcart + m1 + m2)*x.diff(t)**2 + (1/2)*(m1*l**2 + m2*l**2 + I)*theta.diff(t)**2
    + (1/2)*(m2*l2**2 + I2)*theta2.diff(t)**2 + (m1*l + m2*l)*x.diff(t)*theta.diff(t)*sp.cos(theta)
    + m2*l2*x.diff(t)*theta2.diff(t)*sp.cos(theta2) + m2*l*l2*sp.cos(theta-theta2)*theta.diff(t)*theta2.diff(t)
    )

U = m1*g*l*sp.cos(theta) + m2*g*(l*sp.cos(theta) + l2*sp.cos(theta2))

L = T - U

EL_x = sp.Eq(
    sp.diff(sp.diff(L, x.diff(t)), t) - sp.diff(L, x),
    F - B_cart_drag * x.diff(t)
    )

EL_theta = sp.Eq(
    sp.diff(sp.diff(L, theta.diff(t)), t) - sp.diff(L, theta),
    -T_drag
    )

EL_theta2 = sp.Eq(
    sp.diff(sp.diff(L, theta2.diff(t)), t) - sp.diff(L, theta2),
    -T_drag
    )

#move everything to one side of the equation (set equal to 0)
Eq1 = EL_x.lhs - EL_x.rhs
Eq2 = EL_theta.lhs - EL_theta.rhs
Eq3 = EL_theta2.lhs - EL_theta2.rhs
#set up symbols for x2div and theta2div
x2div,theta2div,theta22div = sp.symbols('x2div theta2div theta22div', real = True)
#substitute in new symbols in place of accelerations
Eq1 = Eq1.subs({sp.Derivative(x,t,2):x2div, sp.Derivative(theta,t,2):theta2div, sp.Derivative(theta2,t,2):theta22div})
Eq2 = Eq2.subs({sp.Derivative(x,t,2):x2div, sp.Derivative(theta,t,2):theta2div, sp.Derivative(theta2,t,2):theta22div})
Eq3 = Eq3.subs({sp.Derivative(x,t,2):x2div, sp.Derivative(theta,t,2):theta2div, sp.Derivative(theta2,t,2):theta22div})

#solve equations for accelerations
Subs = sp.solve([Eq1, Eq2, Eq3], [x2div, theta2div, theta22div], simplify = True)

#substitute in symbols for each state
Y1,Y2,Y3,Y4,Y5,Y6 = sp.symbols('Y1 Y2 Y3 Y4 Y5 Y6')
StateSubs = {theta:Y1, sp.Derivative(theta,t):Y2, theta2:Y3, sp.Derivative(theta2,t):Y4, x:Y5, sp.Derivative(x,t):Y6}
F1 = Subs[theta2div].subs(StateSubs)
F2 = Subs[theta22div].subs(StateSubs)
F3 = Subs[x2div].subs(StateSubs)

#combine into a matrix (non-linear state space model), and create state matrix (Y)
NonLinMod = sp.Matrix([Y2, F1, Y4, F2, Y6, F3])
Y = sp.Matrix([Y1, Y2, Y3, Y4, Y5, Y6])

#linearize model
A = NonLinMod.jacobian(Y)
B = NonLinMod.diff(F)
C = sp.Matrix([ #manual input
    [1, 0, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0],
    [0, 0, 0, 0, 1, 0],])
D = sp.Matrix([0, 0, 0]) #manual input

#Substitute in values
Equilibrium = {Y1:0,Y2:0,Y3:0,Y4:0,Y5:0,Y6:0} #all states are 0 at equilibrium
A = A.subs(vals).subs(Equilibrium)
B = B.subs(vals).subs(Equilibrium)

#convert to a ss system, and discretize
ssCont = ctrl.ss(A, B, C, D)
ssDisc = ctrl.c2d(ssCont, Ts)


# #Testing
# # Debugging Block
# print("Is A finite:", np.isfinite(ssDisc.A).all())
# print("Is B finite:", np.isfinite(ssDisc.B).all())
# 
# # Check for controllability
# C_mat = ctrl.ctrb(ssDisc.A, ssDisc.B)
# print("Controllability Rank:", np.linalg.matrix_rank(C_mat))
# 
# # Eigenvalue check
# print("Eigenvalues of A (continuous):", np.linalg.eigvals(ssCont.A))
    

#-----TUNING VALUES-----#
#calculate lqr and kalman filter values
#K, S, E = ctrl.lqr(A, B, sp.diag(10,2,1,1), 0.5)
Kd, Sd, Ed = ctrl.dlqr(ssDisc, sp.diag(8,1,8,1,8,0.5), 1)
#QN and RN are multiplied/divided by Ts to discretize them. MATLAB does this internally
Ld, Pd, Edkalm = ctrl.dlqe(ssDisc.A, sp.diag(1,1,1,1,1,1), ssDisc.C, Ts*sp.diag(0.2,0.001,0.2,0.001,0.2,0.001), (0.01/Ts)*sp.diag(1,1,1))



# --- FINAL FILE OUTPUT SECTION ---
try:
    print("Writing to file...")
    
    # 1. Calculate the absolute path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(script_dir, "control_matrices.txt")

    # 2. Use the 'file_path' variable here, NOT the string 'control_matrices.txt'
    with open(file_path, 'w') as f:
        f.write("=== CONTROL SYSTEM DESIGN DATA ===\n\n")
        
        f.write("--- Matrix A (Linearized) ---\n")
        f.write(np.array2string(np.array(A.tolist(), dtype=float), precision=ArrayPrec) + "\n\n")
        
        f.write("--- Matrix B ---\n")
        f.write(np.array2string(np.array(B.tolist(), dtype=float), precision=ArrayPrec) + "\n\n")

        f.write("--- Matrix C ---\n")
        f.write(np.array2string(np.array(C.tolist(), dtype=float), precision=ArrayPrec) + "\n\n")

        f.write("--- Matrix D ---\n")
        f.write(np.array2string(np.array(D.tolist(), dtype=float), precision=ArrayPrec) + "\n\n")
        
        f.write("--- LQR Gain (Kd) ---\n")
        f.write(np.array2string(np.array(Kd), precision=ArrayPrec) + "\n\n")
        
        f.write("--- Kalman Gain (Ld) ---\n")
        f.write(np.array2string(np.array(Ld), precision=ArrayPrec) + "\n")
        
    # 3. Print the full path so you can click it in the terminal
    print(f"Successfully saved to: {file_path}")

except Exception as e:
    print(f"FAILED to write file. Error: {e}")

print(A)
