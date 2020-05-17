classdef ConstraintManager < handle

    properties (SetAccess = private)
        C_equality;
        C_inequality;
    end

    methods (Access = public)

        function obj = ConstraintManager()
            obj.C_equality = containers.Map();
            obj.C_inequality = containers.Map();
        end

        function add_eq_constraint(obj, key, C_eq1, C_eq2)
            obj.C_equality(key) =  {C_eq1, C_eq2};
        end

        function add_ineq_constraint(obj, key, C_neq1, C_neq2)
            obj.C_inequality(key) =  {C_neq1, C_neq2};
        end

        function remove_eq_constraint(obj, key)
            remove(obj.C_equality, key);
        end

        function remove_ineq_constraint(obj, key)
            remove(obj.C_inequality, key);
        end

    end
end

