Ts = 0.004;
%% Physical constants
g = 9.81;                       % gravity acceleration [m/sec^2]
%% Physical parameters
m = 0.035 ;						% wheel weight [kg]
R = 0.0325;						% wheel radius [m]
Jw = m * R^2 / 2;				% wheel inertia moment [kgm^2]
M = 0.3392;                        % body weight [kg]
W = 0.068;						% body width [m]
%D = 0.11;						% body depth [m]
h = 0.130;						% body height [m]
%L = 0.102;						% distance of the center of mass from the wheel axle [m]
m_b = 0.226;        % battery box mass [kg]
D_b = 0.1016;       % box depth (front-back) [m]
h_b = 0.0259;     % box height [m]
L_b = 0.09795;        % axle to box centre [m]

% Stem / chassis (vertical part)
m_s = M - m_b;    % remaining body mass [kg]
d_s = 0.029;        % stem depth (front-back) [m]
l_s = 0.085;        % stem length from axle to bottom of box [m]
L_s = l_s/2;      % axle to stem centre [m]

L    = (m_b*L_b + m_s*L_s) / M ;                        % combined CoM height
Jpsi = m_b*(D_b^2 + h_b^2)/12 + m_b*(L_b - L)^2 ...      % box about own centre + shift
     + m_s*(d_s^2 + l_s^2)/12 + m_s*(L - L_s)^2 ;      

%Jpsi = M * L^2 / 3;				% body pitch inertia moment [kgm^2]
Jphi = M * (W^2 + D_b^2) / 12;	% body yaw inertia moment [kgm^2]
%fm = 0.0022;					% friction coefficient between body & DC motor
fw = 0;           				% friction coefficient between wheel & floor
%% Motors parameters
%Jm = 1e-5;						% DC motor inertia moment [kgm^2]
%Rm = 3;						% DC motor resistance [Om]
%Kb = 0.006;						% DC motor back EMF constant [Vsec/rad]
%Kt = 8.17e-4;						% DC motor torque constant [Nm/A]
%n = 48;							% Gear ratio
K_PWM = 42.5;                  % Volts to PWM value coefficient [1/V] assuming 8 bit pwm

Psi0 = deg2rad(5);                    % Initial value to make disturbance
 %% Motor parameters — TT gearmotor (130-size DC motor, 48:1 gearbox)
 n       = 48;                          % gear ratio
 Rm      = 3;                           % armature resistance [Ohm] (consistent with speed data; verify with multimeter)
 V_rated = 6;                           % rated voltage [V]
 I0      = 0.15;                        % no-load current at 6 V [A]          (datasheet)
 w0      = 200*2*pi/60;                 % no-load output speed at 6 V [rad/s] (datasheet)
 T_stall = 0.8*9.81/100;                % stall torque at output, 6 V [Nm]   (datasheet)

 Kb  = (V_rated - I0*Rm) / (n*w0);      % back-EMF constant, motor side [V s/rad]   ~0.0055
 Kt  = Kb;                              % SI units: Kt [Nm/A] = Kb [V s/rad]
 eta = T_stall*Rm / (n*Kt*V_rated);     % gearbox efficiency implied by stall torque ~0.15
 fm  = n*Kt*I0 / w0;                    % equivalent viscous friction from no-load current ~0.0019
 Jm  = 2e-7;                            % rotor inertia [kgm^2], estimated as 1/2*m*r^2 (m~8 g, r~6.5 mm)
 V_dz = 0;
%% Motor parameters - 120:1 gearmotor with quadrature encoder (datasheet)
% n        = 120;                        % gear ratio
% V_rated  = 6;                          % rated voltage [V]
% I0       = 0.17;                       % no-load current at 6 V [A]
% w0       = 160*2*pi/60;                % no-load output speed at 6 V [rad/s]
% I_stall  = 2.8;                        % stall current at 6 V [A]
% T_stall  = 0.8*9.81/100;               % stall torque at output [Nm] (0.8 kgf.cm)
% 
% Rm  = V_rated / I_stall;               % armature resistance from stall current [Ohm]
% Kb  = (V_rated - I0*Rm) / (n*w0);      % back-EMF constant, motor side [V s/rad]
% Kt  = Kb;                              % SI units: Kt [Nm/A] = Kb [V s/rad]
% eta = T_stall / (n*Kt*I_stall);        % gearbox efficiency implied by stall torque
% fm  = n*Kt*I0 / w0;                    % equivalent viscous friction [Nm s/rad]
% Jm  = 2e-7;       
%%  Battery box (top)
