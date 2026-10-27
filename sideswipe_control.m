%% sideswipe_control.m — LQR with integral action on theta
% sideswipe_plant;                      % builds s1 and s3 from your parameters
% QQ = eye(7)
% RR = eye(2)
% K_lqr= lqr(s4,QQ,RR)

%% sideswipe_control.m - LQR and PID controller design for Sideswipe
% State vector (s4): x = [theta_int theta psi theta_dot psi_dot phi phi_dot]'
% Inputs:            u = [Vl Vr]'
% Both controllers use the same control law as the Simulink model:
%     u = K*(ref - x)      (equivalent to u = -K*x when ref = 0)

sideswipe_plant;                       % builds s1, s2, s3, s4

%% 1. Reference inputs (used by the REF block in Simulink)
theta_dot_ref = 0;                     % forward speed reference [rad/s]
phi_dot_ref   = 0;                     % turning rate reference [rad/s]
cotroller = 1
%% 2. Open-loop checks
disp('Open-loop poles, pitch/forward (s1):'); disp(eig(s1.A))
disp('Open-loop poles, yaw (s2):');           disp(eig(s2.A))
fprintf('Controllability rank of s4: %d (need 7)\n', rank(ctrb(s4.A, s4.B)))

%% 3. Discretise at the controller sample time
s4d = c2d(s4, Ts, 'zoh');

%% 4. LQR weights using Bryson's rule
% Each weight = 1 / (largest acceptable value)^2
thetaint_max = 7.5;                      % [rad s]
theta_max    = deg2rad(90);           % [rad]   quarter wheel turn (~0.2 m)
psi_max      = deg2rad(3);             % [rad]   robot must stay close to upright
thetadot_max = deg2rad(360);           % [rad/s]
psidot_max   = deg2rad(90);            % [rad/s]
phi_max      = deg2rad(90);            % [rad] these dont matter
phidot_max   = deg2rad(180);           % [rad/s]
V_max        = 4;                      % [V]     

x_max = [thetaint_max theta_max psi_max thetadot_max psidot_max phi_max phidot_max];
QQ = diag(1 ./ x_max.^2);
RR = diag(1 ./ [V_max V_max].^2);

K_lqr = dlqr(s4d.A, s4d.B, QQ, RR);    % 2x7
disp('K_lqr ='); disp(K_lqr)

%% 5. PID gains
% Balance and position loop (same voltage to both motors)
%Kp_psi = 80;     % [V/rad]
%Kd_psi = 8;      % [V s/rad]
%Kp_th  = 1.5;    % [V/rad]
%Kd_th  = 1.5;    % [V s/rad]
%Ki_th  = 0.5;    % [V/(rad s)]
% Yaw loop (opposite voltages to the two motors)
Kp_phi = 21;      % [V/rad] whats 9 + 10?
Kd_phi = 0.67;    % [V s/rad]
%Cascaded PID
Kp_x  = 0.02;            % lean per unit wheel angle        [rad/rad]
Kd_x  = 0.02;            % lean per unit wheel speed        [rad/(rad/s)]
Ki_x  = 0.005;               % lean per unit integral of angle  [rad/(rad s)]

Kp_psi_c = 80;           % [V/rad]
Ki_psi_c = 0;            % [V/(rad s)]
Kd_psi_c = 8;            % [V s/rad]  (gain on gyro rate)
psi_ref_max = deg2rad(10);


%% 6. Closed-loop stability check (all magnitudes must be < 1)
Acl_lqr = s4d.A - s4d.B*K_lqr;

fprintf('LQR: largest closed-loop pole magnitude = %.4f\n', max(abs(eig(Acl_lqr))))


% Equivalent continuous-time poles (easier to interpret in the report)
disp('LQR closed-loop poles [1/s]:'); disp(log(eig(Acl_lqr))/Ts)




%% 8. Linear response to an initial tilt (no saturation or quantisation)
% x0 = [0 0 Psi0 0 0 0 0]';
% cl_lqr = ss(Acl_lqr, zeros(7,2), [eye(7); -K_lqr], zeros(9,2), Ts);
% [y_lqr, t] = initial(cl_lqr, x0, 5);


% figure;
% subplot(3,1,1)
% plot(t, rad2deg(y_lqr(:,3)), t, rad2deg(y_pid(:,3))); grid on
% ylabel('\psi [deg]'); legend('LQR','PID'); title('Response to initial tilt')
% subplot(3,1,2)
% plot(t, 100*R*y_lqr(:,2), t, 100*R*y_pid(:,2)); grid on
% ylabel('Position [cm]')
% subplot(3,1,3)
% plot(t, y_lqr(:,8), t, y_pid(:,8)); grid on
% ylabel('v_l [V]'); xlabel('Time [s]')



%% Plant matrices used by the Simulink model (nominal values)
Bd   = [zeros(2); inv(E)];        % disturbance input: [F_theta; F_psi]
A1p  = A1;   B1p = [B1 Bd];       % pitch/forward plant, inputs [Vl Vr Ftheta Fpsi]
A2p  = A2;   B2p = B2;            % yaw plant
dist = [0 0 0; 1 0 0];            % no disturbance: [time Ftheta Fpsi]
V_sat = 6;                        % motor voltage limit [V]
%% Sensor model (used by Real_out when real_model = 1)
enc_res    = 0.1875;      % encoder resolution [deg]        from df robot
gyro_res   = 0.061;   % gyro resolution [deg/s]         - from BNO055 datasheet
gyro_bias  = 0;      % constant gyro offset [deg/s]
gyro_drift = 0;      % rate of change of offset [deg/s per s]
gyro_noise = 0;      % gyro noise, RMS [deg/s]
accel_noise = 0;
accel_bias = 0;
tau_cf  = 2;                      % filter time constant (s)
alpha_comp    = tau_cf/(tau_cf + Ts);     % = 0.992
est_mode =2;                       %for filter tuning
% theta_dot_f(k) = theta_dot_f(k-1) + a*(theta_dot_raw(k) - theta_dot_f(k-1))
tau_v = 0.01;            % [s] filter time constant
a_v   = Ts/(tau_v + Ts);
n_delay = 0;