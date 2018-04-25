classdef inheritance_test < Constraint & ControlConstraint
    properties (Access = public)
        a
    end
    methods (Access = public)
        function obj = inheritance_test(X, U)
            obj@ControlConstraint(10);
            obj@Constraint(X, U)
            obj.a = 1;
        end
            
    end
        
end