import numpy as np
import sympy as sp
import control as ctrl
import time
import os
import dill


print("program started")

#define symbols and symbol properties
t,g,l,l2,m1,m2,mcart,B_cart_drag = sp.symbols('t g l l2 m1 m2 mcart B_cart_drag', positive = True)
theta = sp.Function('theta')(t) #define theta as a function of t
theta2 = sp.Function('theta2')(t)
x = sp.Function('x')(t) #define x as a function of t
I,I2,F,T_drag = sp.symbols('I I2 F T_drag', real = True)

print("symbols defined")

#Sample Period
Ts = 1/200

ArrayPrec = 5 #how many decimals are sent to the Text file

#0.5N rolling friction

#actual system values
length = 0.25
mweight = 0.03 #weight of pendulum weight
mrod = 0.072 #weight of pendulum arm
lval = 0.25 - 0.085#(mweight*length+mrod*(length/2))/(mweight+mrod) #in meters (center of mass)  #FIX THIS YOU IDIOT
Ival = 0.001688 #((mweight + (mrod/3))*lval**2) #rotational inertia
m1val = 0.204#mweight + mrod #in kg

# length2 = 0.25 #probably not needed for anything
length2 = 0.2
#mrod2 = 0.051
#mweight2 = 0.022
mtotal2 = 0.09
lval2 = 0.11 #length to center of mass
Ival2 = 0.00075978 #((mweight2 + (mrod2/3))*length2**2) #rotational inertia

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

# Check if we already have the symbolic model
if not os.path.exists("symbolic_model.pkl"):
    print("No .pkl, deriving equations ...")
        
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

    print("Basic equations defined")

    #move everything to one side of the equation (set equal to 0)
    Eq1 = EL_x.lhs - EL_x.rhs
    Eq2 = EL_theta.lhs - EL_theta.rhs
    Eq3 = EL_theta2.lhs - EL_theta2.rhs
    print("Equations set equal to 0")
    #set up symbols for x2div and theta2div
    x2div,theta2div,theta22div = sp.symbols('x2div theta2div theta22div', real = True)
    #substitute in new symbols in place of accelerations
    Eq1 = Eq1.subs({sp.Derivative(x,t,2):x2div, sp.Derivative(theta,t,2):theta2div, sp.Derivative(theta2,t,2):theta22div})
    Eq2 = Eq2.subs({sp.Derivative(x,t,2):x2div, sp.Derivative(theta,t,2):theta2div, sp.Derivative(theta2,t,2):theta22div})
    Eq3 = Eq3.subs({sp.Derivative(x,t,2):x2div, sp.Derivative(theta,t,2):theta2div, sp.Derivative(theta2,t,2):theta22div})
    print("abstract symbols substituted in for 2nd derivatives")

    #solve equations for accelerations
    Subs = sp.solve([Eq1, Eq2, Eq3], [x2div, theta2div, theta22div], simplify = True)
    print("Equations solved for derivatives")
    
    #substitute in symbols for each state
    Y1,Y2,Y3,Y4,Y5,Y6 = sp.symbols('Y1 Y2 Y3 Y4 Y5 Y6')
    StateSubs = {theta:Y1, sp.Derivative(theta,t):Y2, theta2:Y3, sp.Derivative(theta2,t):Y4, x:Y5, sp.Derivative(x,t):Y6}
    F1 = Subs[theta2div].subs(StateSubs)
    F2 = Subs[theta22div].subs(StateSubs)
    F3 = Subs[x2div].subs(StateSubs)
    print("State symbols substituted in")

    #combine into a matrix (non-linear state space model), and create state matrix (Y)
    NonLinMod = sp.Matrix([Y2, F1, Y4, F2, Y6, F3])
    Y = sp.Matrix([Y1, Y2, Y3, Y4, Y5, Y6])

    print("Non-linear model created")

    #linearize model
    A = NonLinMod.jacobian(Y)
    B = NonLinMod.diff(F)

    print("linearization completed")

    #Substitute in values
    Equilibrium = {Y1:0,Y2:0,Y3:0,Y4:0,Y5:0,Y6:0} #all states are 0 at equilibrium
    A = A.subs(vals).subs(Equilibrium)
    B = B.subs(vals).subs(Equilibrium)

    print("Values and equilibrium points substituted in")
    
    # Save the solve results and the Jacobians
    with open("symbolic_model.pkl", "wb") as f:
        dill.dump({'A_sym': A, 'B_sym': B, 'Y': Y, 'F': F}, f)
else:
    print("Loading stored data...")
    with open("symbolic_model.pkl", "rb") as f:
        data = dill.load(f)
        A, B, Y, F = data['A_sym'], data['B_sym'], data['Y'], data['F']

C = sp.Matrix([ #manual input
    [1, 0, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0],
    [0, 0, 0, 0, 1, 0],])
D = sp.Matrix([0, 0, 0]) #manual input

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
start = time.time()
Kd, Sd, Ed = ctrl.dlqr(ssDisc, sp.diag(5,1,4,1,3,1), 5)
end = time.time()
print("LQG gain matrix calculated", (end-start)*1000)

#QN and RN are multiplied/divided by Ts to discretize them. MATLAB does this internally
start = time.time()
Ld, Pd, Edkalm = ctrl.dlqe(ssDisc.A, sp.diag(1,1,1,1,1,1), ssDisc.C, Ts*sp.diag(0.2,0.001,0.2,0.001,0.2,0.001), (0.01/Ts)*sp.diag(1,1,1))
end = time.time()
print("Kalman matrices generated", (end-start)*1000)

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