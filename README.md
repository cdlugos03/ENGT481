# ENGT481

Repository for ENGT 480/481 Senior Design Capstone (Triple Inverted Pendulum project)

**Raspberry Pi setup**
 - The python libraries are installed in a venv virtual environment called 'PythonLibraries'.
 - path: /home/engt481/PythonLibraries
 - Thonny is set up to use the python libraries from this environment, if you want to use a different editor you'll need to set this up.
 - Activation command for running scripts in terminal (from engt481 directory): source PythonLibraries/bin/activate
 - Deactivation command: deactivate

**Single pendulum LQG (Linear Quadratic Gaussian) setup:**
 - Run communication test program and hold up pendulum to get a rough estimate of the value when straight up.
 - If the angle is close to the rollover point (near 0 or 16383), rotate the pendulum about the shaft so the rollover point will be near pointing down.
 - replace the value for the 'correction' variable with the measured value for straight up.
 - start with the 'ANGLE CORRECTION', and 'ANGLE TUNING' lines enabled, and run the program until the pendulum centers near the middle (it may oscillate a bit).
 - Comment out the 'ANGLE CORRECTION' line, and replace the 'correction' variable with the final printed correction value. Run again until the pendulum is centered.
 - Comment out the 'ANGLE TUNING' lines, and run again with the new correction value. Make sure that it centers itself very close to the starting point.
 - The first pendulum angle should be tuned at this point, and be usable for the any of the control scripts.

**Double pendulum LQG setup:**
 - Use the single pendulum script first to tune the first pendulum angle
 - Run the communication test program, and let the pendulums hang. Record the second pendulum angle to use for the 'correction2' variable value.
 - The control script should be ready to run at this point.
 - To adjust tuning, or physical measurements of the pendulum parts, update the values in the math script, and run it. this will update the control_matrices.txt file.
 - To adjust the abstract dynamics, make sure to delete the .pkl file, and then run the math script to generate a file for the new dynamics. This will take a long time to run (only when re-generating the .pkl file), so run this on a personal computer. it will take from 15-30 minutes to calculate.
 - The pendulum will likely not stabilize on the first test, so methodically mess with the 'correction2' value until the pendulum is able to stay upright (Increasing the value brings it right, decreasing brings it left)

**Triple Pendulum LQG:**
 
 We had not attempted this as of writing this. Good luck! This will require a more sophisticated control system than linear LQG. Here are some ideas we had:
  - Could try re-linearizing the system every time step to approximate a non-linear system with LQG. (Raspberry pi may not be powerful enough if not well optimized)
 - Could try model predictive control (MPC). (Raspberry pi may not be powerful enough if not well optimized)
 - Could try a machine learning approach. 


**Things to be careful of:**
 - printing to terminal takes a surprising amount of time. Minimize print statements in the control loop, and round float values
 - Make sure to power down the raspberry pi properly. If you just cut the power, it could cause it to corrupt the github.


**Helpful resources:**
 - Matlab pendulum simulation tutorial:
https://www.mathworks.com/help/symbolic/derive-and-simulate-cart-pole-system.html
 - Link to double pendulum paper where we found dynamic equations:
https://www.researchgate.net/publication/348468205_Modeling_and_Control_of_a_Double_Inverted_Pendulum_using_LQR_with_Parameter_Optimization_through_GA_and_PSO

**Suggestions:**
 - Need a way to tune the angles more precisely. Maybe a program could be designed to help tune the second pendulum angle more easily and precisely?
 - PID corrections could help reduce/eliminate drifting in the rail position over time.
 - Bearing friction and air resistance are not included in the model. These may need to be modeled for the triple pendulum.
