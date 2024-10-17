% getAB - Function to obtain A and B matrices from initial flight
% condition, using small perturbation method
function [A, B] = getAB(IC_filename, FlightData, Xg)

% Import initial conditions
load(IC_filename)

% Initialise arrays
A = zeros(12,12);   % A Matrix
B = zeros(12,5);    % B Matrix

% Calculate trim Xdot and Udot
Xdot_0 = zeros(12,1);
Xdot_0 = getXdot(X0, Xg, Xdot_0, U0, FlightData);
Udot_trim = getXdot(X0,Xg,Xdot_0,U0,FlightData);


% Apply perturbation to each of the 12 state variables
delta = 1e-7;   % Small perturbation amount

for i = 1:12
    % Reset state vector to initial condition
    X = X0;

    % Apply small perturbation
    X(i) = X(i) + delta;

    % Calculate Xdot
    Xdot = getXdot(X, Xg, zeros(12,1), U0, FlightData);

    A(:,i) = (Xdot - Xdot_0)/delta;
end


% Apply perturbation to each of the 5 control variables
for j = 1:5
    U = U0;
    U(j) = U0(j) + delta;
    Udot_0 = getXdot(X0, Xg, Xdot_0, U, FlightData);
    B(:,j) = (Udot_0-Udot_trim)/delta;
end

end