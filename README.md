# ENGT481

Repository for ENGT 480/481 Senior Design Capstone (Triple Inverted Pendulum project)

**Things to be careful of:**
 - printing to terminal take a surprising amount of time. Minimize print statements in the control loop, and round float values

**Single pendulum LQG (Linear Quadratic Gaussian) setup:**
 - Run communication test program and hold up pendulum to get a rough estimate of the value when straight up.
 - If the angle is close to the rollover point (near 0 or 16383), rotate the pendulum about the shaft so the rollover point will be near pointing down.
 - replace the value for the 'correction' variable with the measured value for straight up.
 - start with the 'ANGLE CORRECTION', and 'ANGLE TUNING' lines enabled, and run the program until the pendulum centers near the middle (it may oscillate a bit).
 - Comment out the 'ANGLE CORRECTION' line, and replace the 'correction' variable with the final printed correction value. Run again until the pendulum is centered.
 - Comment out the 'ANGLE TUNING' lines, and run again with the new correction value. Make sure that it centers itself very close to the starting point.
 - The first pendulum angle should be tuned at this point, and be usable for the any of the control scripts.




**Possible issues:**
 - Angles may not be spot on, causing inaccuracies.
 - Linear LQG may just not be powerful enough for the double pendulum. May need to switch to a non-linear method.
 - bearing friction and air resistance are not included in the model. These may need to be modeled for accuracy.

**Suggestions:**
 - Need a way to tune the angles precisely
 - PID corrections could help reduce/eliminate drifting
 - Could try gain scheduling or re-linearizing the system every time step to approximate a non-linear system with LQG. (Rasperry pi may not be powerful enough)
 - Could try model predictive control (MPC) (Rasperry pi may not be powerful enough)

