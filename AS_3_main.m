clc
clear
close all

%%%% PLOTTING SETTINGS %%%%
% Set default plot settings
set(0,'defaultAxesFontSize',13)
set(0,'defaultAxesFontName','Times New Roman')
set(0,'defaulttextInterpreter','latex')
set(groot,'DefaultLineLineWidth',1.05)
set(0,'DefaultAxesGridAlpha',0.15)

% Default Axes Colours
myred           = [216 30 49]/255;
myblue          = [27 99 157]/255;
myblack         = [0 0 0]/255;
mygreen         = [0 128 0]/255;
mycyan          = [2 169 226]/255;
myyellow        = [251 194 13]/255;
mygray          = [89 89 89]/255;

set(groot,'defaultAxesColorOrder',[myblack;myred;myblue;mygreen;myyellow;mycyan;mygray]);

figresolution = 600;    % Set figure export resolution (dpi)

%%%%%%%%%% SETUP %%%%%%%%%%

    % Load FlightData Structure
    FlightData = FlightDataAircraft1;
    FlightData.Geometric = FlightData.Geo;
    FlightData.Inertial = FlightData.I;
    FlightData.Propeller = FlightData.Prop;
    FlightData = rmfield(FlightData,["Geo","I","Prop"]);
    
    % Import initial conditions
    IC_filename = 'ICsAircraft1_120knts_400ft';
    load(IC_filename)

    % Set gust vector
    Xg = zeros(6,1);    % Gust Vector

    % Compute initial (trim) Xdot
    Xdot0 = zeros(12,1);
    Xdot0 = getXdot(X0, Xg, Xdot0, U0, FlightData);

%% LINEARISATION
%  Linearises the aircraft dynamics using small perturbation motion about
%  the IC trim state. Code copied from AERO4560 Task Group 1

% Get A and B matrices for initial condition using small
% perturbation method
[A, B] = getAB(IC_filename,FlightData,Xg);

% Remove flap from control vector and B matrix
B(:,5) = [];
U0(5)  = [];

% Obtain X, U, A, B, for longitudinal motion
[Alon, Blon, Xlon0, Xlondot0, Ulon0] = getLon(A,B,X0,Xdot0,U0);

% Set initial condition to ZERO for state and control vectors
% Why? - Once system is linearised, linearised system is for perturbations
% ABOUT the equilibrium condition. Hence all ICs become zero.
% When plotting results, then add original ICs back in to get actual state
% values
Xlondot0 = zeros(4,1);
Xlon0 = zeros(4,1);
Ulon0 = zeros(2,1);

% Define D matrix (direct transmission matrix)
D = [0,0];

%% Calculate transfer function (Task 1)
    % Pitch rate response to elevator deflection
    [de2q_num, de2q_den] = ss2tf(Alon,Blon(:,2),[0,0,1,0],D(:,2));
    de2q_SYSpol = tf(de2q_num, de2q_den);
    [z, p, k] = ss2zp(Alon,Blon(:,2),[0,0,1,0],D(:,2));
    de2q_SYSzpk = zpk(z,p,k);

%% PITCH RATE CONTROLLER DESIGN
% Add integrator before using sisotool - The open loop TF has a
% differentiator. Add integrator and then cancel out by using minreal().
% Must do this outside of sisotool
s = tf('s');
de2q_SYSzpk_integrator = minreal(de2q_SYSzpk/s);

%% Import pitch rate controller
% VERSION 3
load('qController_V3_Block.mat');


%% Calculate elevator to vertical speed TF
Vtrim = sqrt(X0(1)^2 + X0(2)^2 + X0(3)^2);
C_vs = [0 -1 0 Vtrim];
    % Vertical speed response to elevator deflection
    [de2vs_num, de2vs_den] = ss2tf(Alon,Blon(:,2),C_vs,D(:,2));
    de2vs_SYSpol = tf(de2vs_num, de2vs_den);
    [z, p, k] = ss2zp(Alon,Blon(:,2),C_vs,D(:,2));
    de2vs_SYSzpk = zpk(z,p,k);

% Calculate sensitivity function
Sq = kq_V3/(1+kq_V3*de2q_SYSzpk_integrator);
qc2vs_SYSzpk = kq_V3*Sq*de2vs_SYSzpk;



