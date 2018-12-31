# Robust Model Predictive Control Using Tube
This repository includes examples for the tube model predictive control (tube-MPC)[1, 2] as well as the generic model predictive control (MPC) written in MATLAB.

## Requirement

1) optimization_toolbox (matlab)<br>
2) control_toolbox (matlab)<br>
3) Multi-Parametric Toolbox 3 (open-source and freely available at http://people.ee.ethz.ch/~mpt/3/)


## Usage
See `example/example_tubeMPC.m` and `example/example_MPC.m` for the tube-MPC and generic MPC, respectively. Note that every inequality constraint here is expressed as a covex set. For example, the costraints on state Xc is specified as a rectangular, which is costructed with 4 vertexes. When considering a 1-dim input Uc, Uc will be specified by min and max value (i.e. u∊[u_min, u_max]), so it will be constructed by 2 vertexes. For more detail, please see the example codes.

## Disturbance invariant set 
I think one may get stuck at computation of what paper [1] called "disturvance invariant set". The disturbance invariant set is an infinte [Minkowski addition](https://en.wikipedia.org/wiki/Minkowski_addition) `Z = W + Ak*W + Ak^2*W...`, where + denotes Minkowski addition. Obtaining this analytically is impossible, and then, what comes ones's mind first may approximation of that by truncation, namely just `Z_approx = W+Ak*W...+Ak^n*W`. This approximation, however, leads to `Z_approx ⊂ Z`, which means `Z_approx` is not disturbance invariant. So, we must multiply it by some parameter `alpha` and get `Z_approx = alpha*(W+Ak*W...+Ak^n*W)` so that `Z ⊂ Z_approx` is guaranteed. (see `src/LinearSystem.m` for this implementation). The order of truncation and `alpha` are tuning parameters, and I chose values that are large enough. But, if you want to choose them in more sophisticated way, paper [3] will be useful. Also, `example/example_dist_inv_set.m` may help you understand how disturbance the invariant set works.

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



# Reference
[1] Langson, Wilbur, et al. "Robust model predictive control using tubes." Automatica 40.1 (2004): 125-133.
[2] Mayne, David Q., María M. Seron, and S. V. Raković. "Robust model predictive control of constrained linear systems with bounded disturbances." Automatica 41.2 (2005): 219-224.
[3] Rakovic, Sasa V., et al. "Invariant approximations of the minimal robust positively invariant set." IEEE Transactions on Automatic Control 50.3 (2005): 406-410.
