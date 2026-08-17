clear;
clc;
close all;

%% =========================================================
%  V1 CLOSED-LOOP SPEED LIMITER
%  Two-wheel electric drivetrain prototype
% ==========================================================

%% Motor parameters
J  = 0.01;       % kg.m^2 - total rotational inertia
b  = 0.001;      % N.m.s - viscous friction
Kt = 0.05;       % N.m/A - motor torque constant
Ke = 0.05;       % V.s/rad - back EMF constant
R  = 1.0;        % Ohm - armature resistance
L  = 0.5;        % H - armature inductance

V_supply = 12;   % Motor supply voltage

%% =========================================================
% SPEED LIMITER
% ==========================================================

RPM_limit = 400;          % Maximum permitted RPM
requested_RPM = 600;      % Requested speed from throttle

% Actual target is never allowed above the limit
target_RPM = min(requested_RPM, RPM_limit);

%% =========================================================
% PI CONTROLLER
% ==========================================================

Kp = 0.013;
Ki = 0.006;

%% =========================================================
% SIMULATION SETTINGS
% ==========================================================

dt = 0.001;       % Simulation step
T  = 5;           % Simulation duration
t  = 0:dt:T;

%% =========================================================
% STATE VARIABLES
% ==========================================================

omega = zeros(size(t));     % rad/s
current = zeros(size(t));   % A
rpm = zeros(size(t));       % RPM
pwm = zeros(size(t));       % 0-1
voltage = zeros(size(t));   % V
error_signal = zeros(size(t));

integral_error = 0;

%% =========================================================
% LOAD DISTURBANCE
% ==========================================================

load_torque = zeros(size(t));

% Apply mechanical load from 3 seconds onward
load_torque(t >= 3) = 0.03;     % N.m

%% =========================================================
% MAIN SIMULATION LOOP
% ==========================================================

for k = 1:length(t)-1

    %% Measure wheel speed
    rpm(k) = omega(k) * 60 / (2*pi);

    %% Speed error
    error = target_RPM - rpm(k);
    error_signal(k) = error;

    %% -----------------------------------------------------
    % PI CONTROLLER
    % ------------------------------------------------------

    % Unsaturated controller output
    control = Kp * error + Ki * integral_error;

    % Saturate to available motor voltage
    control_sat = max(0, min(V_supply, control));

    % ------------------------------------------------------
    % Anti-windup
    % Only integrate when saturation is not forcing the
    % controller in the wrong direction
    % ------------------------------------------------------

    if ~(control >= V_supply && error > 0) && ...
       ~(control <= 0 && error < 0)

        integral_error = integral_error + error * dt;
    end

    %% PWM duty cycle
    pwm(k) = control_sat / V_supply;

    %% Motor voltage
    voltage(k) = control_sat;

    %% ------------------------------------------------------
    % ELECTRICAL MOTOR DYNAMICS
    % ------------------------------------------------------

    di = (voltage(k) ...
        - R*current(k) ...
        - Ke*omega(k)) / L;

    current(k+1) = current(k) + di*dt;

    %% ------------------------------------------------------
    % MECHANICAL MOTOR DYNAMICS
    % ------------------------------------------------------

    domega = (Kt*current(k) ...
            - b*omega(k) ...
            - load_torque(k)) / J;

    omega(k+1) = omega(k) + domega*dt;

end

%% Final values
rpm(end) = omega(end) * 60/(2*pi);
pwm(end) = pwm(end-1);
voltage(end) = voltage(end-1);
error_signal(end) = target_RPM - rpm(end);

%% =========================================================
% PERFORMANCE METRICS
% ==========================================================

peak_RPM = max(rpm);

overshoot = max(0, ...
    (peak_RPM - target_RPM) / target_RPM * 100);

steady_state_RPM = mean(rpm(end-500:end));

steady_state_error = ...
    target_RPM - steady_state_RPM;

fprintf('\n========================================\n');
fprintf('V1 SPEED LIMITER RESULTS\n');
fprintf('========================================\n');

fprintf('Target RPM          : %.2f\n', target_RPM);
fprintf('Peak RPM            : %.2f\n', peak_RPM);
fprintf('Overshoot           : %.2f %%\n', overshoot);
fprintf('Final RPM           : %.2f\n', rpm(end));
fprintf('Steady-state RPM    : %.2f\n', steady_state_RPM);
fprintf('Steady-state error  : %.2f RPM\n', steady_state_error);

%% =========================================================
% FIGURE 1 — RPM RESPONSE
% ==========================================================

figure;

plot(t, rpm, 'LineWidth', 1.5);
hold on;

yline(target_RPM, '--', 'Speed Limit');

xline(3, ':', 'Load Applied');

xlabel('Time (s)');
ylabel('Wheel Speed (RPM)');
title('V1 Closed-Loop Speed Limiter');

legend('Actual RPM', ...
       'Maximum RPM', ...
       'Load Disturbance');

grid on;

%% =========================================================
% FIGURE 2 — PWM OUTPUT
% ==========================================================

figure;

plot(t, pwm*100, 'LineWidth', 1.5);
hold on;

xline(3, ':', 'Load Applied');

xlabel('Time (s)');
ylabel('PWM Duty Cycle (%)');
title('Controller Output');

grid on;

%% =========================================================
% FIGURE 3 — ERROR
% ==========================================================

figure;

plot(t, error_signal, 'LineWidth', 1.5);
hold on;

yline(0, '--');

xline(3, ':', 'Load Applied');

xlabel('Time (s)');
ylabel('Speed Error (RPM)');
title('Speed Control Error');

grid on;

%% =========================================================
% FIGURE 4 — MOTOR CURRENT
% ==========================================================

figure;

plot(t, current, 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Motor Current (A)');
title('Motor Current');

grid on;