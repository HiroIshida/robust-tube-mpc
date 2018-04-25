# Robust Model Predictive Control Using Tube
This repository includes example sets for generic MPC, disturbance-invariant MPC[1], and Tube-MPC[2] in MATLAB.

# Requirement

1) optimization_toolbox (matlab)<br>
2) control_toolbox (matlab)<br>
3) Multi-Parametric Toolbox 3 (open-source and freely available at http://people.ee.ethz.ch/~mpt/3/)


# How to run
Here, I'd like to explain how to test the Tube-MPC. Define system dynamics: x(k+1) = Ax(k)+Bu(k)
```
A = [1 1; 0 1]; B = [0.5; 1]; 
```
and the quadratic cost: x'Qx+u'Ru.
```
Q = diag([1, 1]); R = 0.1;
```
Specify the polyhedral vertexes of the state and input constraints: Xc and Uc, and disturbance set: W.
```
Xc_vertex = [2, -2; 2 2; -10 2; -10 -2];
Uc_vertex = [1; -1];
W_vertex = [0.15, 0.15; 0.15, -0.15; -0.15, -0.15; -0.15, 0.15];
```
and then make polyhedral objects as follows.
```
Xc = Polyhedron(Xc_vertex);
Uc = Polyhedron(Uc_vertex);
W = Polyhedron(W_vertex);
```
where I used the Polyhedron class in MPT3. There are several other ways for constructing a polyhedron, which is well described in the 
<a href="http://control.ee.ethz.ch/~mpt/3/Geometry/Sets">
documentation
</a> of the MPT3. Initial state: x_init and prediction horizon: N are specified as follows.
```
x_init = [-7; -2]; N = 10;
```
Then run the simulation as follows.
```
tmpc = TubeModelPredictiveControl(A, B, Q, R, Xc, Uc, W, N, x_init);
tmpc.simulation();
tmpc.show_result();
```
You can finally have the following results.
<div align="center">
<img src="https://raw.githubusercontent.com/HiroIshida/robust-tube-mpc/master/fig/sample2.jpg" alt="none" title="sample2" width="700">
</div>
Now that you can see that the green nominal trajectory starting from the bottom left of the figure and surrounding a "tube". The blue plot means the real trajectory affected by the disturbance. You can see that this real trajectory never stick out from the "tube", and is robustly guided into the region [Xf-Z]. 
</html>
<div align="center">
<img src="https://raw.githubusercontent.com/HiroIshida/robust-tube-mpc/master/fig/sample1.jpg" alt="none" title="sample1" width="700">
</div>
Protecting that in a 3-dim fashion with a minus-time axis in the z-direction, you can easily get why we call that "tube". 

Let me give some important details. The red region [Xc] that contains the pink region [Xc-Z] is the state constraint that we give first. However, considering the uncertainty, the tube-MPC designs the nominal trajectory to be located inside [Xc-Z], which enables to put "tube" around the nominal trajectory such that the tube is also contained in [Xc-Z]. Of course, the input sequence associated with the nominal trajectory is inside of [Uc-KZ]. If there is no disturbance, putting a terminal constraint [Xf] (set it to Maximum Positively Invariant Set, here) guarantees the stability and feasibility. However, considering the disturbance we have to put the terminal constraint [Xf-Z] instead. Once the terminal constraint [Xf-Z] is satisfied, in spite of the disturbance, the following trajectory will be stable around the origin by LQR feedback without violating any constraints.

# Note
1) I made this program to study MPC, object-oriented Programming (actually it is my first time to make a class from scratch!), and about GitHub. So, I would love to listen to any comments and advise.<br>
2) I am planning to add Homothetic Tube MPC[3] in a month.

# Reference
[1] Mayne, D. Q., and W. Lanston. "Robustifying model predictive control of constrained linear systems." Electronics Letters 37.23 (2001): 1422-1423. <br>
[2] Mayne, David Q., María M. Seron, and S. V. Raković. "Robust model predictive control of constrained linear systems with bounded disturbances." Automatica 41.2 (2005): 219-224.<br>
[3] Raković, Saša V., et al. "Homothetic tube model predictive control." Automatica 48.8 (2012): 1631-1638.<br>