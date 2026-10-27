 %% Load parameters
sideswipe_params;
%% Helping variables
%alpha = n * Kt / Rm; % amount of torqu out per volt applied combines gearbox ratio and torque constant with motor esistance
%beta = n * Kt * Kb / Rm + fm; % damping kinda motors resistance to motion due to back emf
alpha = eta * n * Kt / Rm;                   % torque at wheel per volt
beta  = eta * n^2 * Kt * Kb / Rm + fm;       % back-EMF (motor-shaft speed = n x wheel speed) + friction
E = [(2*m+M)*R^2 + 2*Jw + 2*n^2*Jm   M*L*R - 2*n^2*Jm;
      M*L*R - 2*n^2*Jm               M*L^2 + Jpsi + 2*n^2*Jm]; % mass/inertia matrix 
F = 2*[beta+fw   -beta; 
       -beta     beta];% damping maatrix
G = [0   0; 
     0  -M*g*L]; % gravity/stiffness matrix
H = [alpha   alpha; 
    -alpha  -alpha];% input matrix - hoow the motor voltages effect the system
I = m * W^2 / 2 + Jphi + (Jw + n^2 * Jm) * W^2 / (2 * R^2); % yaw efective momebnt of inertia 
J = W^2 / (2 * R^2) * (beta + fw); % yaw damping 
Ky = W / (2 * R) * alpha; % how motor voltage differences effect yaw
%% State space matrices
A1 = [0 0   1 0;
	  0 0   0 1;
     -E\G  -E\F]; 
B1 = [0 0;
	  0 0;
	  E\H];
C1 = eye(4);
D1 = zeros(4, 2);
A2 = [0   1
	  0  -J/I];
B2 = [0    0
	 -Ky/I  Ky/I];
C2 = eye(2);
D2 = zeros(2);
%% State space models
s1 = ss(A1, B1, C1, D1);
s1.StateName = {'theta', 'psi', 'theta_dot', 'psi_dot'};
s1.InputName = {'Vl', 'Vr'};
s1.OutputName = {'theta', 'psi', 'theta_dot', 'psi_dot'};
s2 = ss(A2, B2, C2, D2);
s2.StateName = {'phi', 'phi_dot'};
s2.InputName = {'Vl', 'Vr'};
s2.OutputName = {'phi', 'phi_dot'};
%% Add a fifth state - integral of theta
s0 = ss(1/tf('s'));
s0.StateName = 'theta_int';
s0.OutputName = 'theta_int';
s3 = append(s0, s1);
s3.A(1, 2) = 1;
s3(:, 1) = [];
%% Add phi and phi_dot states (from s2 system)
s4 = append(s3, s2);
s4.B(end, [1 2]) = s4.B(end, [3 4]);
s4(:, [3 4]) = [];