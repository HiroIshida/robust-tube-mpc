# robust-tube-mpc
This repository now includes the basic MPC and disturbance-invariant MPC (explaind in Langson 2004). 
I am planning to add Tube-MPC(Mayne 2005) soon and Homothetic Tube-MPC(Rakovic 2012) in few months.
At the current stage, the code is messy because it is my first time to define a class from scratch and to study model predictive control, but of course, it will be fixed later!

The green line is the nominal trajectory and the blue line is the actual one.
![sample](https://user-images.githubusercontent.com/38597814/39091101-3bbfe1e8-4628-11e8-9fda-576222b99120.jpg)
You can get the similar result if you run example.m (Te actual trajectory depends on the random noise)
