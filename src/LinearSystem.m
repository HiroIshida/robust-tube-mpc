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
            obj.A = A;
            obj.B = B;
            obj.Q = Q;
            obj.R = R;
            obj.nx = size(A, 1);
            obj.nu = size(B, 2);

            [K_tmp, obj.P] = dlqr(obj.A, obj.B, obj.Q, obj.R);
            obj.K = -K_tmp;
            obj.Ak = (obj.A+obj.B*obj.K);
        end

        function x_new = propagate(obj, x, u) 
            x_new = obj.A * x + obj.B * u;
        end

        function Xmpi = compute_MPIset(obj, Xc, Uc) 
            [F, G, nc] = convert_Poly2Mat(Xc, Uc);
            Fpi = @(i) (F+G*obj.K)*obj.Ak^i;
            Xpi = @(i) Polyhedron(Fpi(i), ones(size(Fpi(i), 1), 1));
            Xmpi = Xpi(0);
            i= 0;
            while(1) % 
                i = i + 1;
                Xmpi_tmp = and(Xmpi, Xpi(i));
                if Xmpi_tmp == Xmpi
                    break;
                else
                    Xmpi = Xmpi_tmp;
                end
            end
        end
    end
end

