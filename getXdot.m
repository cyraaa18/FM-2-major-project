% getXdot - Function to calculate state rates via iteration of input
% state

function [Xdot] = getXdot(X, Xg, Xdot, U, FlightData)

iter = 1;
error = 1;
tol = 1e-32;   % Tolerance for convergence criterion

while error > tol
    % Calculate force and moment coefficients
    [ForceCoeff, MomentCoeff] = aero4560_aero(X, Xg, Xdot, U, FlightData);
    [Xdot_o] = aero4560_motion(X, ForceCoeff, MomentCoeff, FlightData);

    % Compute error
    error = norm((Xdot_o - Xdot)/Xdot_o);

    % Print iteration count and error
    % fprintf("Iteration: %.f     Error: %.15f\n", iter, error);

    % Write new Xdot for next iteration
    Xdot = Xdot_o;
    iter = iter + 1;
end

end