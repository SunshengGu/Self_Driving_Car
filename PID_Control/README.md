## Effect of P I D Components
The P component opposes the CTE directly. If the car drifts away from the center of the lane, it tries to steer the car back to the the center of the lane. But the P term has the tendency of over-correcting the CTE and generates CTE in the opposite direction. I played around with the Kp value, and when this coefficient is larger, the car tend to swerve around more. The CTE eventually diverges and the car eventually drove off the road.

The D term helps solve the over-correcting problem of the P term. The D term is a function of the difference between the current and previous CTE. So if the CTE is decreasing, the D term will try to steer the car in the opposite direction as the P term does, thus reducing oscillation. I observed that as Kd got larger, oscillation was less severe.

The I term helps reducing accumulative error, since it is a function of the sum of CTE. One thing worth noting is that Ki must be much smaller than Kd and Kp, because the error term for Ki is relatively large. When I initiated Ki as 0.1, the car drove off the track pretty quickly. 

## Choice of Hyperparameters
The final Kp, Ki, Kd values were calculated using the twiddle algorithm taught in class. But the initial Kp, Ki, Kd values and the dp values were tuned manually. I selected the initial values as such: {Kp, Ki, Kd} = {0.05, 0.0009, 0.8}, dp = {0.01, 0.0001, 0.1}, and twiddle was perform every 100 times the main function ran. The final parameters are {Kp, Ki, Kd} = {0.071636, 0.0013309, 0.96628} after 15 mins of twiddling. 
