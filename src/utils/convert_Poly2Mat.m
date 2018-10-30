function [F, G, nc] = convert_Poly2Mat(X, U)
    % Constraints set X and U in Polyhedron form -> F, G matrix form
    % nc; number of contraint forced by Xc and Uc
    % F; G; % constraints for state and input: Fx+Gu<=1, where 1 is a voctor
    poly2ineq = @(poly) poly.A./repmat(poly.b, 1, size(poly.A, 2));
    F_tmp = poly2ineq(X);
    G_tmp = poly2ineq(U);
    if numel(F_tmp)==0
        F_tmp = zeros(0, X.Dim);
    end
    if numel(G_tmp)==0
        G_tmp = zeros(0, U.Dim);
    end
    F = [F_tmp; zeros(size(G_tmp, 1), X.Dim)];
    G = [zeros(size(F_tmp, 1), U.Dim); G_tmp];
    nc = size(F, 1);
end
