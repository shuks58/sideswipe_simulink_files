 %%  Simulation tests for Sideswipe
%
% Runs the four comparison tests on the Simulink model 'system_model':
%   Test 1  - recovery from a 5 deg initial tilt, and the largest recoverable tilt
%   Test 2a - a push on the body, and the largest push recovered
%   Test 2b - a constant slope, and the steepest slope held
%   Test 3  - a step in the commanded wheel speed, and the largest step followed
%   Test 4  - realistic conditions (BNO055 datasheet values)
%
% Each test is run once for the LQR controller and once for the cascaded PID
% controller under identical conditions. Results are printed as tables and the
% figures are saved to a 'figures' folder next to the model.
 
clear r1 r2a r2b r3 r4 m1 m2a m2b m3


%% Setup
make_plots = false;
sideswipe_control;                  % builds the plant, K_lqr and all controller settings
mdl = 'system_model';
load_system(mdl);
real_model = 1;                     % use the realistic sensor and actuator models
% Nominal physical parameters, collected in one struct
% (used by plant_matrices when the plant needs to be rebuilt with different values)
% p0 = struct('m',m, 'R',R, 'Jw',Jw, 'M',M, 'W',W, 'D',D, 'L',L, 'Jpsi',Jpsi, ...
            % 'Jphi',Jphi, 'fm',fm, 'fw',fw, 'n',n, 'Rm',Rm, 'Kt',Kt, 'Kb',Kb, ...
            % 'eta',eta, 'Jm',Jm, 'g',g);

% Disturbance input matrix: generalised forces [F_theta; F_psi]
Bd = [zeros(2); inv(E)];

% Baseline settings shared by every test. Each run copies these and changes
% only what that run needs. The field names must match the Simulink variables

base = struct( ...
    'A1p', A1, 'B1p', [B1 Bd], 'A2', A2, 'B2', B2, ...   % nominal plant
    'Psi0',          0, ...                                % start upright
    'dist',          [0 0 0; 1 0 0], ...                   % no disturbance
    'V_sat',         6, ...                                % 6 V motor limit
    'R',             R, ...
    'enc_res',       enc_res, ...                          % realistic encoder resolution
    'gyro_res',      gyro_res, ...
    'accel_noise',   0, ...                                % no accelerometer errors
    'accel_bias',    0, ...
    'gyro_bias',     0, ...                                % no gyroscope errors
    'gyro_drift',    0, ...
    'gyro_noise',    0, ...
    'theta_dot_ref', 0, ...                                % no motion command
    'phi_dot_ref',   0, ...
    'cotroller',     1, ...                                % 1 = LQR, 2 = cascaded PID
    'real_model',    1, ...
    'V_dz',          0, ...
    'n_delay',       0);                                   % no motor deadband

% The two controllers being compared
controller_type = [1 2];                     % 1 = LQR, 2 = cascaded PID
names           = {'LQR', 'Cascaded PID'};
gains           = {K_lqr, K_lqr};            % only used by the LQR; ignored by the cascade

%% TEST 1 - 5 deg initial tilt, and the largest recoverable tilt
T1        = 5;                               % run length [s]
tilt_test = 5;                               % initial tilt for the main run [deg]
 
clear m1
for c = 1:2
    b = settings_for(base, controller_type(c));
 
    % Main run
    r1{c} = run_case(mdl, gains{c}, b, struct('Psi0', deg2rad(tilt_test)), T1);
 
    % Largest initial tilt the controller recovers from (bisection, 0-40 deg)
    recovers_from = @(tilt) recovers(mdl, gains{c}, b, struct('Psi0', deg2rad(tilt)), T1);
    tilt_max(c)   = find_limit(recovers_from, 0, 40, 0.1);
 
    r = r1{c};
    m1(c) = struct( ...
        'ts_s',         r.ts, ...                     % pitch settling time
        'ts_x_s',       r.ts_x, ...                   % position settling time
        'min_psi_deg',  min(rad2deg(r.x(:,2))), ...   % largest undershoot
        'travel_cm',    r.travel_cm, ...
        'final_cm',     r.final_cm, ...
        'Vpeak',        r.Vpeak, ...
        'sat_pct',      r.sat_pct, ...
        'effort',       r.effort, ...
        'ise_psi',      r.ise_psi, ...
        'max_tilt_deg', tilt_max(c));
end
 
disp('TEST 1 - 5 deg initial tilt');
show_table(m1, names);
f = plot_compare(r1, names, R, 'Test 1: 5^\circ initial tilt, 6 V limit');
%save_fig(f, 'test1');

%% TEST 2a - Push on the body
T2a    = 5;                                  % run length [s]
F_push = 1;                                  % push force for the main run [N]
 
% Force F applied at the centre of mass for 0.1 s, starting at t = 1 s
push = @(F) [0    0    0;
             1    F*R  F*L;
             1.1  0    0];
 
clear m2a
for c = 1:2
    b = settings_for(base, controller_type(c));
 
    % Main run
    r2a{c} = run_case(mdl, gains{c}, b, struct('dist', push(F_push)), T2a);
 
    % Largest push the controller recovers from (bisection, 0-10 N)
    recovers_from = @(F) recovers(mdl, gains{c}, b, struct('dist', push(F)), T2a);
    push_max(c)   = find_limit(recovers_from, 0, 10, 0.02);
 
    r      = r2a{c};
    pos_cm = 100*R*r.x(:,1);
    m2a(c) = struct( ...
        'peak_psi_deg', r.peak_psi, ...
        'recover_s',    r.ts - 1, ...         % measured from the start of the push
        'recover_pos',  r.ts_x - 1, ...
        'max_fwd_cm',   max(pos_cm), ...
        'max_back_cm',  min(pos_cm), ...
        'final_cm',     r.final_cm, ...
        'Vpeak',        r.Vpeak, ...
        'sat_pct',      r.sat_pct, ...
        'effort',       r.effort, ...
        'ise_psi',      r.ise_psi, ...
        'max_push_N',   push_max(c));
end
 
fprintf('TEST 2a - %.1f N push for 0.1 s\n', F_push);
show_table(m2a, names);
f = plot_compare(r2a, names, R, sprintf('Test 2a: %.1f N push for 0.1 s', F_push));
%save_fig(f, 'test2a')

%% TEST 2b - Slope (constant disturbance from t = 1 s)
T2b        = 10;                             % run length [s]
slope_test = 3;                              % slope for the main run [deg]
 
% Gravity forces on a slope of gam degrees, applied from t = 1 s
slope = @(gam) [0  0                           0;
                1  -g*sind(gam)*(2*m + M)*R    -g*sind(gam)*M*L];
 
clear m2b
for c = 1:2
    b = settings_for(base, controller_type(c));
 
    % Main run
    r2b{c} = run_case(mdl, gains{c}, b, struct('dist', slope(slope_test)), T2b);
 
    % Steepest slope the controller can hold (bisection, 0-30 deg)
    holds_on = @(gam) holds(mdl, gains{c}, b, struct('dist', slope(gam)), T2b);
    slope_max(c) = find_limit(holds_on, 0, 30, 0.1);
 
    r         = r2b{c};
    pos_cm    = 100*R*r.x(:,1);
    last_sec  = r.t > r.t(end) - 1;
    m2b(c) = struct( ...
        'max_back_cm',   min(pos_cm), ...
        'final_cm',      r.final_cm, ...
        'lean_deg',      rad2deg(mean(r.x(last_sec,2))), ...   % steady lean
        'hold_V',        mean(mean(r.u(last_sec,:))), ...      % holding voltage
        'Vpeak',         r.Vpeak, ...
        'sat_pct',       r.sat_pct, ...
        'max_slope_deg', slope_max(c));
end
 
fprintf('TEST 2b - %g deg slope\n', slope_test);
show_table(m2b, names);
%f = plot_compare(r2b, names, R, sprintf('Test 2b: %g^\\circ slope from t = 1 s', slope_test));
%save_fig(f, 'test2b')

%% TEST 3 - Reference tracking (speed step at t = 1 s)
T3     = 6;                                  % run length [s]
v_step = 2;                                  % commanded wheel speed [rad/s]
 
clear m3
for c = 1:2
    b = settings_for(base, controller_type(c));
 
    % Main run
    r3{c} = run_case(mdl, gains{c}, b, struct('theta_dot_ref', v_step), T3);
 
    % Largest speed step the controller can follow (bisection, 0-15 rad/s)
    follows = @(v) tracks(mdl, gains{c}, b, v, T3);
    v_max(c) = find_limit(follows, 0, 15, 0.1);
 
    m = tracking_metrics(r3{c}, v_step, R);
    m.max_step_rad_s = v_max(c);
    m.max_step_cm_s  = 100*R*v_max(c);
    m3(c) = m;
end
 
fprintf('TEST 3 - speed step of %.1f rad/s (%.1f cm/s)\n', v_step, 100*R*v_step);
show_table(m3, names);
f = plot_tracking(r3, names, v_step, R);
%save_fig(f, 'test3')

%% TEST 4 - Realistic conditions (typical sensor noise, motor deadband and loop delay)
T4         = 10;                                   % run length [s]
tilt_test4 = 5;                                    % initial tilt [deg]
last5      = @(r) r.t > r.t(end) - 5;              % last 5 s of a run

% Typical values (IMU calibrated, so no offsets)
gyro_noise_typ = 0.1;                              % BNO055 typical gyro noise [deg/s RMS]
acc_noise_typ  = rad2deg(150e-6*sqrt(62.5));       % 150 ug/sqrt(Hz) at 62.5 Hz -> 0.068 deg RMS
V_dz_typ       = I0*Rm;                            % motor deadband estimate, 0.45 V
n_delay_typ    = 2;                                % 2 samples = 8 ms (BNO055 default filtering)

noise = struct('gyro_noise', gyro_noise_typ, 'accel_noise', acc_noise_typ);
cases = {
    'Nominal',             struct()
    'Sensor noise only',   noise
    'Deadband only',       struct('V_dz', V_dz_typ)
    'Delay only',          struct('n_delay', n_delay_typ)
    'All combined',        with_field(with_field(noise, 'V_dz', V_dz_typ), 'n_delay', n_delay_typ)
};
n_cases = size(cases, 1);
r4 = cell(n_cases, 2);

fprintf('\nTEST 4 - realistic conditions, %g deg initial tilt, %d s run\n', tilt_test4, T4);
fprintf('%-20s %-13s %5s %10s %10s %10s %10s %10s %8s\n', 'Case', 'Controller', 'Fell', ...
        'Min psi', 'RMS psi', 'p-p psi', 'p-p pos', 'Final pos', 'Effort');
for k = 1:n_cases
    for c = 1:2
        b = settings_for(base, controller_type(c));
        r = run_case(mdl, gains{c}, b, with_field(cases{k,2}, 'Psi0', deg2rad(tilt_test4)), T4);
        r4{k,c} = r;

        late    = last5(r);
        psi_deg = rad2deg(r.x(late,2));
        pos_cm  = 100*R*r.x(late,1);
        psi_rms = sqrt(trapz(r.t(late), psi_deg.^2) / 5);   % time-weighted RMS [deg]

        fprintf('%-20s %-13s %5d %9.2f° %9.3f° %9.3f° %7.2f cm %7.2f cm %8.2f\n', ...
                cases{k,1}, names{c}, r.fell, min(rad2deg(r.x(:,2))), psi_rms, ...
                max(psi_deg) - min(psi_deg), max(pos_cm) - min(pos_cm), r.final_cm, r.effort);
    end
end

% Largest recoverable tilt: nominal vs all combined (relaxed test, allows a small limit cycle)
fprintf('\nTEST 4 - maximum recoverable tilt (deg)\n');
for k = [1 n_cases]
    for c = 1:2
        b = settings_for(base, controller_type(c));
        stays = @(tilt) stays_up(mdl, gains{c}, b, with_field(cases{k,2}, 'Psi0', deg2rad(tilt)), T4);
        fprintf('%-20s %-13s %6.1f\n', cases{k,1}, names{c}, find_limit(stays, 0, 40, 0.1));
    end
end

f = plot_compare(r4(n_cases,:), names, R, 'Test 4: realistic conditions (noise, deadband, delay)');
%save_fig(f, 'test4')

%% functions 
function b = settings_for(base, controller)
    % Copy the baseline settings and select the controller (1 = LQR, 2 = cascade)
    b = base;
    b.cotroller = controller;
end
 
function s = with_field(s, name, value)
    % Return a copy of struct s with one field set
    s.(name) = value;
end
 
function r = run_case(mdl, K, base, changes, T)
    % Run one simulation and return the logged signals and performance metrics.
    %   K       - LQR gain 
    %   base    - baseline settings
    %   changes - settings that differ from the baseline for this run
    %   T       - run length [s]
 
    FALL_ANGLE    = deg2rad(45);     % robot counts as fallen past this pitch
    RECOVER_BAND  = deg2rad(0.5);    % |psi| must stay inside this for the last second
    SETTLE_BAND   = deg2rad(0.25);   % band used for the pitch settling time
    POSITION_BAND = 0.5;             % band used for the position settling time [cm]
 
    % 1. Combine the baseline settings with this run's changes
    v = base;
    changed = fieldnames(changes);
    for i = 1:numel(changed)
        v.(changed{i}) = changes.(changed{i});
    end
 
    % 2. Pass every setting to the model (they apply to this run only)
    in = Simulink.SimulationInput(mdl);
    in = in.setVariable('K_ctrl', K);
    settings = fieldnames(v);
    for i = 1:numel(settings)
        in = in.setVariable(settings{i}, v.(settings{i}));
    end
    in = in.setModelParameter('StopTime', num2str(T));
 
    % 3. Run the simulation
    out = sim(in);
 
    % 4. Extract the logged signals (one row per time step)
    x = out.logsout.get('x_log').Values;     % true states [theta psi theta_dot psi_dot phi phi_dot]
    u = out.logsout.get('u_log').Values;     % applied voltages [v_l v_r]
    r.t = x.Time;
    r.x = as_rows(squeeze(x.Data), numel(r.t));
    %r.u = as_rows(squeeze(u.Data), numel(r.t));
    % The voltage is logged at the controller sample rate, which differs from the
    % plant's time steps, so hold each value until the next sample (zero-order hold)
    u_data = as_rows(squeeze(u.Data), numel(u.Time));
    r.u    = interp1(u.Time, u_data, r.t, 'previous', 'extrap');
 
    try                                      % estimated states, if they were logged
        xe   = out.logsout.get('xe_log').Values;
        r.te = xe.Time;
        r.xe = as_rows(squeeze(xe.Data), numel(r.te));
    catch
        r.te = [];
        r.xe = [];
    end
 
    % 5. Did the robot fall or recover?
    psi     = r.x(:,2);
    pos_cm  = 100*v.R*r.x(:,1);
    r.t_end = r.t(end);
 
    stopped_early = r.t_end < T - 1e-6;      % the model stops the run when the robot falls
    if stopped_early && max(abs(psi)) < FALL_ANGLE
        warning(['Simulation stopped at t = %.2f s but psi never passed 45 deg. ' ...
                 'Check the Stop Simulation condition.'], r.t_end);
    end
    r.fell = stopped_early || any(abs(psi) > FALL_ANGLE);
 
    psi_last_sec = psi(r.t > r.t_end - 1);
    r.recovered  = ~r.fell && ~isempty(psi_last_sec) && max(abs(psi_last_sec)) < RECOVER_BAND;
 
    % 6. Settling times (last time the signal was outside its band)
    r.ts = NaN;                                          % pitch settling time [s]
    i_last = find(abs(psi) > SETTLE_BAND, 1, 'last');
    if r.recovered && ~isempty(i_last)
        r.ts = r.t(i_last);
    end
 
    i_last_x = find(abs(pos_cm) > POSITION_BAND, 1, 'last');
    if r.fell
        r.ts_x = NaN;                        % fell, so it never settled
    elseif isempty(i_last_x)
        r.ts_x = 0;                          % never moved outside the band
    elseif i_last_x == numel(r.t)
        r.ts_x = NaN;                        % still outside the band at the end of the run
    else
        r.ts_x = r.t(i_last_x);
    end
 
    % 7. Other performance metrics
    r.peak_psi   = rad2deg(max(abs(psi)));                % largest pitch [deg]
    r.travel_cm  = 100*v.R*max(abs(r.x(:,1)));            % furthest from start [cm]
    r.final_cm   = 100*v.R*r.x(end,1);                    % final position [cm]
    r.max_abs_cm = max(abs(pos_cm));                      % same as travel_cm [cm]
    r.Vpeak      = max(abs(r.u(:)));                      % peak voltage [V]
 
    at_limit     = double(any(abs(r.u) >= v.V_sat - 0.01, 2));
    r.sat_pct    = 100*trapz(r.t, at_limit)/r.t_end;      % time at the voltage limit [%]
 
    r.effort     = trapz(r.t, sum(r.u.^2, 2));            % control effort [V^2 s]
    r.ise_psi    = trapz(r.t, rad2deg(psi).^2);           % integral of squared pitch [deg^2 s]
    r.iae_x      = trapz(r.t, abs(v.R*r.x(:,1)));         % integral of absolute position [m s]
 
    second_half   = r.t > r.t_end/2;                      % RMS pitch over the second half [deg]
    r.psi_rms_deg = rad2deg(sqrt(trapz(r.t(second_half), psi(second_half).^2) / (r.t_end/2)));
end
 
function A = as_rows(A, n_samples)
    % Make sure each row of a logged signal is one time step
    if size(A,1) ~= n_samples
        A = A';
    end
end
 
function ok = recovers(mdl, K, base, changes, T) %oh no i hope i dont fall
    % True if the robot recovers (does not fall and ends up upright)
    r  = run_case(mdl, K, base, changes, T);
    ok = r.recovered;
end
 
function ok = holds(mdl, K, base, changes, T)
    % True if the robot stays up and settles into a steady lean (slope test)
    r = run_case(mdl, K, base, changes, T);
    if r.fell
        ok = false;
        return
    end
    last_sec = r.t > r.t(end) - 1;
    pitch_rate_small = max(abs(r.x(last_sec,4))) < deg2rad(2);   % steady lean
    not_rolling_away = max(abs(r.x(last_sec,3))) < 0.5;          % wheel speed [rad/s]
    ok = pitch_rate_small && not_rolling_away;
end
 
function ok = tracks(mdl, K, base, v, T) % somebody stop him hes geting away hes getting away
    % True if the robot follows a speed step of v [rad/s] without falling
    r = run_case(mdl, K, base, struct('theta_dot_ref', v), T);
    if r.fell
        ok = false;
        return
    end
    last_sec = r.t > r.t(end) - 1;
    ok = abs(mean(r.x(last_sec,3)) - v) < 0.1*v;   % within 10% of the command
end
function ok = stays_up(mdl, K, base, changes, T)%im reaching the limit of my cycle man
    % True if the robot does not fall and |psi| < 2 deg over the last 2 s
    % (allows a small limit cycle, unlike recovers())
    r  = run_case(mdl, K, base, changes, T);
    ok = ~r.fell && max(abs(r.x(r.t > r.t(end) - 2, 2))) < deg2rad(2);
end
function lim = find_limit(test, lo, hi, tol)
    % Bisection: largest value between lo and hi for which test(value) is true
    while hi - lo > tol
        mid = (lo + hi)/2;
        if test(mid)
            lo = mid;
        else
            hi = mid;
        end
    end
    lim = lo;
end
 
function m = tracking_metrics(r, v, R) % where did you come from where did you go, where did you come from sideswipe eye joe
    % Metrics for a speed step of v [rad/s] at t = 1 s
    t     = r.t;
    theta = r.x(:,1);
    psi   = r.x(:,2);
    speed = r.x(:,3);
 
    after_step = t >= 1;
    last_sec   = t > t(end) - 1;
    theta_ref  = v*max(t - 1, 0);                  % reference wheel angle
 
    m.fell          = r.fell;
    m.dip_cm        = 100*R*min(theta(after_step));          % roll-back (negative = backwards)
    m.rise_s        = NaN;                                   % time to reach 90% of the command
    i90 = find(after_step & speed >= 0.9*v, 1);
    if ~isempty(i90)
        m.rise_s = t(i90) - 1;
    end
    m.overshoot_pct = 100*(max(speed(after_step)) - v)/v;
    m.speed_err     = mean(speed(last_sec)) - v;             % speed error over last second [rad/s]
    m.lag_cm        = 100*R*(theta_ref(end) - theta(end));   % positive = behind the reference
    m.peak_lean_deg = rad2deg(max(abs(psi(after_step))));
    m.lean_deg      = rad2deg(mean(psi(last_sec)));          % lean while driving
    m.Vpeak         = r.Vpeak;
    m.sat_pct       = r.sat_pct;
end
 
function show_table(rows, names)
    % Print a struct array of metrics as a table, one row per controller
    disp(struct2table(rows(:), 'RowNames', names(:)));
end
 
function f = plot_compare(r, names, R, plot_title)%comparison is the theif of joy
    % Pitch, position and left motor voltage for both controllers
    f = figure; try, theme(f,"light"); end
 
    subplot(3,1,1); hold on; grid on
    for c = 1:numel(r), plot(r{c}.t, rad2deg(r{c}.x(:,2))); end
    ylabel('\psi [deg]'); legend(names, 'Location', 'best'); title(plot_title);
 
    subplot(3,1,2); hold on; grid on
    for c = 1:numel(r), plot(r{c}.t, 100*R*r{c}.x(:,1)); end
    ylabel('Position [cm]');
 
    subplot(3,1,3); hold on; grid on
    for c = 1:numel(r), plot(r{c}.t, r{c}.u(:,1)); end
    ylabel('v_l [V]'); xlabel('Time [s]');
end
 
function f = plot_tracking(r, names, v, R)% sounds devious whose plot are we tracking
    % Speed, position, pitch and voltage for the speed-step test
    f = figure;  try, theme(f, 'light'); end
    t0 = r{1}.t;
 
    subplot(4,1,1); hold on; grid on
    for c = 1:numel(r), plot(r{c}.t, 100*R*r{c}.x(:,3)); end
    plot(t0, 100*R*v*(t0 >= 1), 'k--');
    ylabel('Speed [cm/s]'); legend([names, {'Reference'}], 'Location', 'best');
    title(sprintf('Test 3: speed step of %.1f cm/s at t = 1 s', 100*R*v));
 
    subplot(4,1,2); hold on; grid on
    for c = 1:numel(r), plot(r{c}.t, 100*R*r{c}.x(:,1)); end
    plot(t0, 100*R*v*max(t0 - 1, 0), 'k--');
    ylabel('Position [cm]');
 
    subplot(4,1,3); hold on; grid on
    for c = 1:numel(r), plot(r{c}.t, rad2deg(r{c}.x(:,2))); end
    ylabel('\psi [deg]');
 
    subplot(4,1,4); hold on; grid on
    for c = 1:numel(r), plot(r{c}.t, r{c}.u(:,1)); end
    ylabel('v_l [V]'); xlabel('Time [s]');
end
 

 
function save_fig(f, name)
    
    outdir = fullfile(fileparts(which('system_model')), 'figures');
    if ~exist(outdir, 'dir'), mkdir(outdir); end
    exportgraphics(f, fullfile(outdir, [name '.png']), 'Resolution', 300);
end 