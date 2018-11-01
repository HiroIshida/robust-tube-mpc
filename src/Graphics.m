classdef Graphics<handle
    % This is a function collection rather than a class
    % So, All methods are static.
    % plot trajectory and show convex set which are projected from n-dim
    
    methods (Static)
        function show_convex(P, varargin)
            P_reduced = projectPolytope2Plane(P);
            switch numel(varargin)
                case 1
                    fill(P_reduced.V(:, 1), P_reduced.V(:, 2), varargin{1});
                case 3
                    fill(P_reduced.V(:, 1), P_reduced.V(:, 2), varargin{1}, varargin{2}, varargin{3});
            end
            hold on;
        end

        function show_convex_timeslice(P, z_axis, varargin)
            P_reduced = projectPolytope2Plane(P);
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
        
        function show_convex_sliding(S1, S2, h1, h2, varargin)
            % s1 and s2 must has a similarity.
           S1_reduced = projectPolytope2Plane(S1);
           S2_reduced = projectPolytope2Plane(S2);
           V1 = S1_reduced.V;
           V2 = S2_reduced.V;
           num_vert = size(V1, 1);
           for i = 1:num_vert
               plot3([V1(i, 1); V2(i, 1)], [V1(i, 2); V2(i, 2)], [h1, h2], varargin{1})  
           end
           hold on;
        end

        function show_trajectory(x_seq, varargin)
            switch numel(varargin)
                case 1
                    plot(x_seq(1, :), x_seq(2, :), varargin{1});
                case 3
                    plot(x_seq(1, :), x_seq(2, :), varargin{1}, varargin{2}, varargin{3});     
            end
            hold on;
        end
        
        function show_trajectory_timeslice( x_seq, varargin)
            num_xseq = size(x_seq, 2);
            x_seq_3d = [x_seq(1, :); x_seq(2, :); flip(0:(num_xseq-1))];
            switch numel(varargin)
                case 1
                    plot3(x_seq_3d(1, :), x_seq_3d(2, :), x_seq_3d(3, :), varargin{1});
                case 3
                    plot3(x_seq_3d(1, :), x_seq_3d(2, :), x_seq_3d(3, :), varargin{1}, varargin{2}, varargin{3});     
            end
            hold on;
        end
    end
end

function P_projected = projectPolytope2Plane(P)
    vert = P.V;
    x_vert = round(vert(:, 1), 5);
    y_vert = round(vert(:, 2), 5);
    idx = convhull(x_vert, y_vert);
    P_projected = Polyhedron([x_vert(idx), y_vert(idx)]);
end
