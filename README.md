# Robust Model Predictive Control Using Tube
This repository includes examples for the tube model predictive control (tube-MPC)[1] as well as the generic model predictive control (MPC) written in MATLAB.

## Requirement

1) optimization_toolbox (matlab)<br>
2) control_toolbox (matlab)<br>
3) Multi-Parametric Toolbox 3 (open-source and freely available at http://people.ee.ethz.ch/~mpt/3/)


## Usage
See `example/example_tubeMPC.m` and `example/example_MPC.m` for the tube-MPC and generic MPC, respectively.

## Short introduction to the tube MPC
After running `example/example_tubeMPC.m`, you will get the following figure.
<div align="center">
<img src="https://raw.githubusercontent.com/HiroIshida/robust-tube-mpc/master/fig/sample2.jpg" alt="none" title="sample2" width="500">
</div>
Now that you can see that the green nominal trajectory starting from the bottom left of the figure and surrounding a "tube". The blue plot means the real trajectory affected by the disturbance. You can see that this real trajectory never stick out from the "tube", and is robustly guided into the region [Xf-Z]. 
</html>
<div align="center">
<img src="https://raw.githubusercontent.com/HiroIshida/robust-tube-mpc/master/fig/sample1.jpg" alt="none" title="sample1" width="500">
</div>
Protting it in a 3-dim fashion with a minus-time axis in the z-direction, you can easily get why we call that "tube". 

Let me give some important details. The red region [Xc] that contains the pink region [Xc-Z] is the state constraint that we give first. However, considering the uncertainty, the tube-MPC designs the nominal trajectory to be located inside [Xc-Z], which enables to put "tube" around the nominal trajectory such that the tube is also contained in [Xc-Z]. Of course, the input sequence associated with the nominal trajectory is inside of [Uc-KZ]. If there is no disturbance, putting a terminal constraint [Xf] (set it to Maximum Positively Invariant Set, here) guarantees the stability and feasibility. However, considering the disturbance we have to put the terminal constraint [Xf-Z] instead. Once the terminal constraint [Xf-Z] is satisfied, in spite of the disturbance, the following trajectory will be stable around the origin by LQR feedback without violating any constraints.

# TODO
1) better usage  

2) better introduction in README.txt

3) enable arbitrary dimentional system

4) add Homothetic Tube MPC[2].



# Reference
[1] Mayne, David Q., María M. Seron, and S. V. Raković. "Robust model predictive control of constrained linear systems with bounded disturbances." Automatica 41.2 (2005): 219-224.<br>
[2] Raković, Saša V., et al. "Homothetic tube model predictive control." Automatica 48.8 (2012): 1631-1638.<br>
