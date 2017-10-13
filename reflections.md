# PID Controller

The proportional–integral–derivative controller (PID controller) continuously calculates an error value e(t) as the difference between a desired setpoint and a measured process variable and applies a correction based on proportional, integral, and derivative terms (denoted P, I, and D respectively).
In our project the controller adjust the steer angle.

The 3 parts of the controller are:

### P - Proportional

The cross-track error (cte) measures the error from the desired value. For our project cte is the distance between the car and the middle of the road. The P term doesn't take in account the speed of the vehicle. With only this term the controller constanly overshoots left and right.

### D - Derivative

A derivative term does not consider the error (meaning it cannot bring it to zero: a pure D controller cannot bring the system to its setpoint), but the rate of change of error, trying to bring this rate to zero. The derivative term smooths the controller function so the correction term is more stable reducing the overshooting.


### I - Integral

An integral term increases action in relation not only to the error but also the time for which it has persisted. So, if applied force is not enough to bring the error to zero, this force will be increased as time passes. The I part helps to correct the global error to zero. It sums all the errors so if the car tends to go to left or right it corrects the route.

## Hyper-parameters

I implemented the twiddle algorithm in main.cpp. Because it has to run with the simulator it is not possible to runinside a while instruction and the state has to be keeped between calls. It was difficult to tune because the parameters has very different mgnitud orders. If the parameters are not set close to the optimal ones the car goes outside the road and even turns more than 180º. Twiddle algorithm needs and enviroment tolerant to errors and the car simulator it's not. For example, if the integral part (a very small term) is an order of magnitude bigger the car goes outside the road. I had to experiment the parameters to find somestable ones and the apply the twiddle algorithm. The controller is still a bit rough and it would inconfortable to travel inside a car with this implementation. I experimented with close values but I was not able to smooth the oscillations.

