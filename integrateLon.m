%%%%%%%%%%%%%%%%%% TASK 1D %%%%%%%%%%%%%%%%%%
% integrateLon() - Evaluates linearised longitudinal system response to a 
% 1 degree elevator deflection for 1 second
function [Xdot_vec, U_vec, X_vec, Y_vec] = integrateLon(A,B,C,D,X0,Xdot_0,U0,t,h)
% Initialise X and Y response solution arrays
Xdot_vec = zeros(length(X0), length(t));
X_vec = zeros(length(X0), length(t));
Y_vec = zeros(size(C,1), length(t));
X_vec(:,1) = X0;   % Set initial condition for state vector
Xdot_vec(:,1) = Xdot_0;

% Define input signal array
U_vec = repmat(U0,[1,length(t)]);
idx = [  find(t >= 1, 1, "first"),  find(t < 2, 1, "last")  ];

% Calculate initial condition for Y
Y_vec(:,1) = C*X_vec(:,1) + D*U_vec(:,1);

% Add 1 degree elevator deflection for 1 second after t = 1s
deflection = 1 * (pi/180);
U_vec(2,idx(1):end) = U_vec(2,idx(1):end) + deflection;

    % METHOD 1 - RK4 INTEGRATION LOOP
    for i = 1:length(t)-1
        % Define the function f(t, x) = A*x + B*u
        f = @(x, u) A*x + B*u;

        % Compute k1, k2, k3, k4
        k1 = f(X_vec(:, i), U_vec(:, i));
        k2 = f(X_vec(:, i) + 0.5*h*k1, U_vec(:, i));
        k3 = f(X_vec(:, i) + 0.5*h*k2, U_vec(:, i));
        k4 = f(X_vec(:, i) + h*k3, U_vec(:, i+1)); % u(i+1) for the next time step

        % Update the state
        X_vec(:, i+1) = X_vec(:, i) + (h/6) * (k1 + 2*k2 + 2*k3 + k4);

        % Compute the output
        Y_vec(:, i+1) = C*X_vec(:, i+1) + D*U_vec(:, i+1);
    end
    
    % 1D Method 3 - Manual dt integration loop

    % for i = 1:length(t)-1
    %     Xdot_vec(:,i) = A*X_vec(:,i) + B*U_vec(:,i);
    %     X_vec(:,i+1) = X_vec(:,i) + Xdot_vec(:,i)*h;
    %     Y_vec(:,i+1) = C*X_vec(:,i+1) + D*U_vec(:,i+1);
    % end
