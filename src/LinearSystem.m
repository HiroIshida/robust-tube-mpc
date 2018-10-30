classdef LinearSystem < handle
    properties (SetAccess = private)
        % common notation for linear system
        A; B; % dynamics x[k+1] = A*x[k]+B*u[k]
        nx; nu; % dim of state space and input space
        Q; R; % quadratic stage cost for LQR
        K; % LQR feedback coefficient vector: u=Kx
        P; % optimal cost function of LQR is x'*P*x
        Ak %: A + BK closed loop dynamics
    end

    methods (Access = public)
        function obj = LinearSystem(A, B, Q, R)
            obj.A = A
            obj.B = B
            obj.Q = Q
            obj.R = R
            obj.nx = size(A, 1)
            obj.nu = size(B, 2)

            [K_tmp, obj.P] = dlqr(obj.A, obj.B, obj.Q, obj.R);
            obj.K = -K_tmp;
            obj.Ak = (obj.A+obj.B*obj.K);
        end
    end
end

