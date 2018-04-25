classdef ProblemConstraint <handle
    
    properties (SetAccess = protected)
        F; G; % form of Fx+Gu<=1
        Xc; Uc;
        nx;
        nu;
        nc; % number of constraint for Fx+Gu<=1 at each stage (non include initial or terminal or those additional constraints)
    end
    
    methods (Access = public)
        
        function obj = ProblemConstraint(Xc, Uc, varargin)
            if isempty(Xc) == 1
                obj.Xc = Polyhedron();
                obj.nx = varargin{1};
                F_tmp = zeros(0, obj.nx);
            else
                obj.Xc = Xc;
                F_tmp = ProblemConstraint.poly2ineq(obj.Xc);
                obj.nx = size(F_tmp)*[0; 1];
            end
            
            if isempty(Uc) == 1
                obj.Uc = Polyhedron();
                obj.nu = varargin{2};
                G_tmp = zeros(0, obj.nu);
            else
                obj.Uc = Uc;
                G_tmp = ProblemConstraint.poly2ineq(obj.Uc);
                obj.nu = size(G_tmp)*[0; 1];
            end
            
            obj.F = [F_tmp; zeros(size(G_tmp)*[1; 0], obj.nx)];
            obj.G = [zeros(size(F_tmp)*[1; 0], obj.nu); G_tmp];
            obj.nc = size(obj.F)*[1;0];
        end
                        
        function [] = const_minkovski(obj, Xadd, Uadd)
            if isempty(Xadd) ~=1
                obj.Xc = obj.Xc +Xadd;
            end
            if isempty(Uadd) ~=1
                obj.Uc = obj.Uc +Uadd;
            end
            obj.construct_FG();
        end
        
        function cosnst_pontryagin(obj, Xdiff, Udiff)
            if isempty(Xdiff)~=1
                obj.Xc = obj.Xc + Xdiff;
            end
            if isempty(Udiff)~=1
                obj.Uc = obj.Uc + Udiff;
            end
            obj.construct_FG();
        end        
    end
    
    methods(Static)
        
        function Cneq = poly2ineq(poly)
            Cneq  = poly.A./repmat(poly.b, 1, size(poly.A)*[0; 1]);
        end
        
        
            
        end
    end
    
end