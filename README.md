# robust-tube-mpc
This repository now includes the basic MPC and disturbant-invariant MPC (explaind in Langson 2004). 
I am planning to add Tube-MPC(Mayne 2005) soon and Homothetic Tube-MPC(Rakovic 2012) in few months.
At the current stage, the code is messy because it is my first time to define a class from scratch and to study model predictive control, but of course, it will be fixed later! This software requires the installation of the MPT3 toolbox for geometric set operation (Minkovski Sum and Pontryagin Difference). The MPT3 is opensource free software, which is available here-> http://people.ee.ethz.ch/~mpt/3/ 

You can get the following result of disturbance invariant MPC if you run example.m (The actual trajectory depends on the random noise)
The green line is the nominal trajectory and the blue line is the actual one.
![sample](https://user-images.githubusercontent.com/38597814/39091172-d6376ba0-4629-11e8-8439-9e8950766b9d.jpg)
