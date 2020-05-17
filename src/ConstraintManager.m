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

        function add_ineq_constraint(obj, key, C_ineq1, C_ineq2)
            obj.C_inequality(key) =  {C_ineq1, C_ineq2};
        end

        function remove_eq_constraint(obj, key)
            remove(obj.C_equality, key);
        end

        function remove_ineq_constraint(obj, key)
            remove(obj.C_inequality, key);
        end

        function [C_eq1, C_eq2] = combine_all_eq_constraints(obj)
            C_eq_array = values(obj.C_equality);
            C_eq1 = [];
            C_eq2 = [];
            for i = 1:numel(C_eq_array)
                C_eq1 = [C_eq1; C_eq_array{i}{1}];
                C_eq2 = [C_eq2; C_eq_array{i}{2}];
            end
        end

        function [C_ineq1, C_ineq2] = combine_all_ineq_constraints(obj)
            C_ineq_array = values(obj.C_inequality);
            C_ineq1 = [];
            C_ineq2 = [];
            for i = 1:numel(C_ineq_array)
                C_ineq1 = [C_ineq1; C_ineq_array{i}{1}];
                C_ineq2 = [C_ineq2; C_ineq_array{i}{2}];
            end
        end
    end
end

