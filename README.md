# Robust Model Predictive Control Using Tube
This repository includes examples for the tube model predictive control (tube-MPC)[1] as well as the generic model predictive control (MPC) written in MATLAB.

## For user
It would be really really really helpful if you could report some bug or feature request even if it's a tiny one. For those who has github account write an issue or pull request for this repo, and for those who doesn't feel free to send me an email. The address is: `spitfire.docomo@` plus trivial google one.

## Requirement

1) optimization_toolbox (matlab)<br>
2) control_toolbox (matlab)<br>
3) Multi-Parametric Toolbox 3 (open-source and freely available at http://people.ee.ethz.ch/~mpt/3/)


## Usage
See `example/example_tubeMPC.m` and `example/example_MPC.m` for the tube-MPC and generic MPC, respectively. Note that every inequality constraint here is expressed as a convex set. For example, the constraints on the state `Xc` is specified as a rectangular, which is constructed with 4 vertexes. When considering a 1-dim input `Uc`, `Uc` will be specified by min and max value (i.e. `u∊[u_min, u_max]`), so it will be constructed by 2 vertexes. For more detail, please see the example codes.

## Short introduction to the tube MPC
After running `example/example_tubeMPC.m`, you will get the following figure sequence.
![the gif file](/fig/tube_mpc.gif)

Now that you can see that the green nominal trajectory starting from the bottom left of the figure and surrounding a "tube". At each time step, the nominal trajectory (green line) is computed online. 

Let me give some important details. The red region `Xc` that contains the pink region `Xc-Z` is the state constraint that we give first. However, considering the uncertainty, the tube-MPC designs the nominal trajectory to be located inside `Xc-Z`, which enables to put "tube" around the nominal trajectory such that the tube is also contained in `Xc-Z`. Of course, the input sequence associated with the nominal trajectory is inside of `Uc-KZ`. 

## Disturbance invariant set
I think one may get stuck at computation of what paper [1] called "disturbance invariant set". The disturbance invariant set is an infinite [Minkowski addition](https://en.wikipedia.org/wiki/Minkowski_addition) `Z = W ⨁ Ak*W ⨁ Ak^2*W...`, where ⨁ denotes Minkowski addition. Obtaining this analytically is impossible, and then, what comes ones' mind first may be an approximation of that by truncation: `Z_approx = W ⨁ Ak*W...⨁ Ak^n*W`. This approximation, however, leads to `Z_approx ⊂ Z`, which means `Z_approx` is not disturbance invariant. So, we must multiply it by some parameter `alpha` and get `Z_approx = alpha*(W ⨁ Ak*W...⨁ Ak^n*W)` so that `Z ⊂ Z_approx` is guaranteed. (see `src/LinearSystem.m` for this implementation). The order of truncation and `alpha` are tuning parameters, and I chose values that are large enough. But, if you want to choose them in a more sophisticated way, paper [2] will be useful. Also, `example/example_dist_inv_set.m` may help you understand how disturbance the invariant set works. 

## Maximum positively invariant set
I used the maximal positively invariant (MPI) set `Xmpi` as the terminal constraint set. (Terminal constraint is usually denoted as Xf in literature). Book [3] explains the concept of the MPI and algorithm well in section 2.4. `Xmpi` is computed in the constructor of `OptimalControler.m`. Note that the MPI set is computed with `Xc` and `Uc` in the normal MPC setting, but in the tube-MPC the MPI set is computed with `Xc⊖Z`and `Uc⊖Z` instead.

# Reference
[1] Mayne, David Q., María M. Seron, and S. V. Raković. "Robust model predictive control of constrained linear systems with bounded disturbances." Automatica 41.2 (2005): 219-224.
[2] Rakovic, Sasa V., et al. "Invariant approximations of the minimal robust positively invariant set." IEEE Transactions on Automatic Control 50.3 (2005): 406-410.
[3] Kouvaritakis, Basil, and Mark Cannon. "Model predictive control." Switzerland: Springer International Publishing (2016).
