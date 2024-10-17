% getLon() - Takes A, B, X, and U arrays and returns versions for
% longitudinal motion:
% Xlon = [u,w,q,theta]^T
% Ulon = [delta_t, delta_e]^T

function [Alon, Blon, Xlon, Xlondot0, Ulon] = getLon(A, B, X, Xdot0, U)

% Isolate longitudinal components of A and B matrix
Alon = A([1,3,5,8],[1,3,5,8]);
Blon = B([1,3,5,8],[1,2]);

% Isolate longitudinal components of control and state vector
Xlon = X([1,3,5,8]);
Xlondot0 = Xdot0([1,3,5,8]);
Ulon = U([1,2]);

end