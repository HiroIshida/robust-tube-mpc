classdef NeqConstraint<hundle
    % convex to matrix form
    % matrix to convex form 
    properties (GetAccess = public, SetAccess = private)
        Cneq % convex set can be expressed as: Cneq*x<=1
        P 
    end
    
    methods (Access = public)
        function obj = NeqConstraint(arg1)
            if isa(arg1, 'Polyhedron')
                obj.P = arg1;
                obj.Cneq = obj.P.A./repmat(obj.P.b, 1, size(obj.P.A)*[1; 0]);
            else
                obj.Cneq = arg1;
                obj.P = Polyhedron(obj.Cneq, ones())
            end
        end
        
    end
    
end