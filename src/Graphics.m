classdef Graphics<handle
    % This is a functino collection rather than a class
    % plot trajectory and show convex set which are projected from n-dim
    % space to 2-dim space spanned by axis1 and axis2.
    % Users can specify axis1 and axis2.
    
    properties (SetAccess = protected)
        axis1;
        axis2;
    end
    
    methods (Access = public)
        
        function obj = Graphics(varargin)
            switch nargin
                case 0
                    obj.axis1 = 1;
                    obj.axis2 = 2;
                case 2
                    obj.axis1 = varargin{1};
                    obj.axis2 = varargin{2};
                otherwise
                    error('invalid number of argument');
            end
        end
        
        function set_axis(obj, axis1, axis2)
            obj.axis1 = axis1;
            obj.axis2 = axis2;
        end
        
        function show_convex(obj, P, varargin)
            P_reduced = obj.projectPolytope2Plane(P);
            switch numel(varargin)
                case 1
                    fill(P_reduced.V(:, 1), P_reduced.V(:, 2), varargin{1});
                case 3
                    fill(P_reduced.V(:, 1), P_reduced.V(:, 2), varargin{1}, varargin{2}, varargin{3});
            end
            hold on;
        end

        function show_convex_timeslice(obj, P, z_axis, varargin)
            P_reduced = obj.projectPolytope2Plane(P);
            num_vertex = size(P_reduced.V, 1);
            vertex_3d = [P_reduced.V, z_axis*ones(num_vertex, 1)];
            switch numel(varargin)
                case 1
                    fill3(vertex_3d(:, 1), vertex_3d(:, 2), vertex_3d(:, 3), varargin{1});
                case 3
                    fill3(vertex_3d(:, 1), vertex_3d(:, 2), vertex_3d(:, 3), varargin{1}, varargin{2}, varargin{3});
            end
            hold on;
        end
        
        function show_convex_sliding(obj, S1, S2, h1, h2, varargin)
            % s1 and s2 must has a similarity.
           S1_reduced = obj.projectPolytope2Plane(S1);
           S2_reduced = obj.projectPolytope2Plane(S2);
           V1 = S1_reduced.V;
           V2 = S2_reduced.V;
           num_vert = size(V1, 1);
           for i = 1:num_vert
               plot3([V1(i, 1); V2(i, 1)], [V1(i, 2); V2(i, 2)], [h1, h2], varargin{1})  
           end
           hold on;
        end


        function show_trajectory(obj, x_seq, varargin)
            switch numel(varargin)
                case 1
                    plot(x_seq(obj.axis1, :), x_seq(obj.axis2, :), varargin{1});
                case 3
                    plot(x_seq(obj.axis1, :), x_seq(obj.axis2, :), varargin{1}, varargin{2}, varargin{3});     
            end
            hold on;
        end
        
        function show_trajectory_timeslice(obj, x_seq, varargin)
            num_xseq = size(x_seq, 2);
            x_seq_3d = [x_seq(obj.axis1, :); x_seq(obj.axis2, :); flip(0:(num_xseq-1))];
            switch numel(varargin)
                case 1
                    plot3(x_seq_3d(1, :), x_seq_3d(2, :), x_seq_3d(3, :), varargin{1});
                case 3
                    plot3(x_seq_3d(1, :), x_seq_3d(2, :), x_seq_3d(3, :), varargin{1}, varargin{2}, varargin{3});     
            end
            hold on;
        end
    end
    
    methods (Access = protected)
        
        function P_projected = projectPolytope2Plane(obj, P)
            vert = P.V;
            x_vert = round(vert(:, obj.axis1), 5);
            y_vert = round(vert(:, obj.axis2), 5);
            idx = convhull(x_vert, y_vert);
            P_projected = Polyhedron([x_vert(idx), y_vert(idx)]);
        end
    end
    
end
