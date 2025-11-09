%% CSTR Feedback Control System Analysis
%% Student ID: 88
% Date: 2025-11-09
% Author: SanguraKoded

clear all; close all; clc;

%% 1. Define Steady-State Parameters (Based on Student ID '88')
printf('=== CSTR Control System Analysis ===\n\n');

% Student ID last 2 digits: Y = 88
Y = 88;

% Process Parameters
Fi = (Y + 50) / 3600;        % Feed flowrate: kg/h -> kg/s
rho = 1000;                   % Density: kg/m³
Cp = 4200;                    % Specific heat: J/kg°C (converted from 4.2 kJ/kg°C)
V = 700 / 1000;               % Volume: L -> m³
lambda_st = 2000 * 1000;      % Latent heat: J/kg (converted from 2000 kJ/kg)

% Display parameters
fprintf('Process Parameters:\n');
fprintf('  Feed Flowrate (Fi): %.6f kg/s\n', Fi);
fprintf('  Density (rho): %.0f kg/m³\n', rho);
fprintf('  Specific Heat (Cp): %.0f J/kg°C\n', Cp);
fprintf('  Volume (V): %.3f m³\n', V);
fprintf('  Latent Heat (lambda_st): %.0f J/kg\n\n', lambda_st);

%% 2. Calculate Transfer Function Parameters
tau = (rho * Cp * V) / (rho * Cp * Fi);  % Time constant (s)
K = lambda_st / (rho * Cp * Fi);          % Process Gain (°C·s/kg)

fprintf('Transfer Function Parameters:\n');
fprintf('  Time Constant (tau): %.2f seconds\n', tau);
fprintf('  Process Gain (K): %.2f °C·s/kg\n\n', K);

%% 3. Define Open-Loop Transfer Function G_p(s) = K / (tau*s + 1)
numerator = K;
denominator = [tau, 1];
G_p = tf(numerator, denominator);

fprintf('Open-Loop Transfer Function G_p(s):\n');
disp(G_p);

%% 4. Calculate and Display Poles and Zeros
poles_Gp = pole(G_p);
zeros_Gp = zero(G_p);

fprintf('\nPoles of G_p(s): %.6f\n', poles_Gp);
if isempty(zeros_Gp)
    fprintf('Zeros of G_p(s): None\n\n');
else
    fprintf('Zeros of G_p(s): %.6f\n\n', zeros_Gp);
end

%% 5. Plot Pole-Zero Map (Diagram 2)
figure(1);
pzmap(G_p);
grid on;
title('Pole-Zero Map of CSTR Open-Loop Transfer Function', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Real Axis', 'FontSize', 12);
ylabel('Imaginary Axis', 'FontSize', 12);
% Add annotations
hold on;
plot(poles_Gp, 0, 'rx', 'MarkerSize', 15, 'LineWidth', 2);
text(poles_Gp - 0.001, 0.0005, sprintf('  Pole at s = %.6f', poles_Gp), 'FontSize', 10);
hold off;
saveas(gcf, 'pole_zero_map.png');

%% 6. Open-Loop Step Response (Figure 4.1)
step_input_magnitude = Y;  % Step magnitude = 88
figure(2);
[y_step, t_step] = step(step_input_magnitude * G_p, 0:1:5*tau);
plot(t_step, y_step, 'b-', 'LineWidth', 2);
grid on;
title('Open-Loop Step Response', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Time (seconds)', 'FontSize', 12);
ylabel('Output Temperature (°C)', 'FontSize', 12);
% Add steady-state line
steady_state_value = step_input_magnitude * K;
hold on;
plot([0, max(t_step)], [steady_state_value, steady_state_value], 'r--', 'LineWidth', 1.5);
legend('Step Response', sprintf('Steady State = %.2f', steady_state_value), 'Location', 'best');
hold off;
saveas(gcf, 'open_loop_step_response.png');

% Calculate step response characteristics
info_step = stepinfo(step_input_magnitude * G_p);
fprintf('Step Response Characteristics:\n');
fprintf('  Rise Time: %.2f seconds\n', info_step.RiseTime);
fprintf('  Settling Time: %.2f seconds\n', info_step.SettlingTime);
fprintf('  Steady State Value: %.2f\n\n', steady_state_value);

%% 7. Open-Loop Ramp Response (Figure 4.2)
ramp_slope = Y + 3;  % Slope = 91
t_ramp = 0:0.1:5*tau;
% Create ramp input (starts at t=0)
ramp_input = ramp_slope * t_ramp;

figure(3);
% Use lsim for ramp input
[y_ramp, t_ramp_out] = lsim(G_p, ramp_input, t_ramp);
plot(t_ramp_out, ramp_input, 'r--', 'LineWidth', 1.5); hold on;
plot(t_ramp_out, y_ramp, 'b-', 'LineWidth', 2);
grid on;
title('Open-Loop Ramp Response', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Time (seconds)', 'FontSize', 12);
ylabel('Output', 'FontSize', 12);
legend('Input Ramp', 'Output Response', 'Location', 'best');
hold off;
saveas(gcf, 'open_loop_ramp_response.png');

% Calculate steady-state error for ramp
steady_state_error_ramp = ramp_slope * tau;
fprintf('Ramp Response:\n');
fprintf('  Steady-State Error: %.2f\n\n', steady_state_error_ramp);

%% 8. Controller Design - Ziegler-Nichols Tuning
% For first-order system without dead time, use PI controller
% Ziegler-Nichols tuning for first-order process
Kc_ZN = 0.9 / K;              % Proportional gain
tau_I_ZN = 3 * tau;            % Integral time

fprintf('Ziegler-Nichols PI Controller Tuning:\n');
fprintf('  Proportional Gain (Kc): %.4f\n', Kc_ZN);
fprintf('  Integral Time (tau_I): %.2f seconds\n\n', tau_I_ZN);

%% 9. Define Controllers (P, PI, PD, PID)
% P Controller
Kc_P = Kc_ZN;
G_c_P = tf(Kc_P, 1);

% PI Controller
G_c_PI = tf([Kc_ZN * tau_I_ZN, Kc_ZN], [tau_I_ZN, 0]);

% PD Controller
tau_D = tau / 4;  % Derivative time
G_c_PD = tf([Kc_ZN * tau_D, Kc_ZN], 1);

% PID Controller
G_c_PID = tf([Kc_ZN * tau_I_ZN * tau_D, Kc_ZN * tau_I_ZN + Kc_ZN * tau_D, Kc_ZN], ...
             [tau_I_ZN, 0]);

%% 10. Closed-Loop Systems
% Closed-loop transfer functions (assume sensor gain = 1)
G_cl_P = feedback(G_c_P * G_p, 1);
G_cl_PI = feedback(G_c_PI * G_p, 1);
G_cl_PD = feedback(G_c_PD * G_p, 1);
G_cl_PID = feedback(G_c_PID * G_p, 1);

%% 11. Compare Controllers (Figure 4.3)
figure(4);
t_control = 0:0.1:10*tau;
[y_P, ~] = step(G_cl_P, t_control);
[y_PI, ~] = step(G_cl_PI, t_control);
[y_PD, ~] = step(G_cl_PD, t_control);
[y_PID, ~] = step(G_cl_PID, t_control);

plot(t_control, y_P, 'r-', 'LineWidth', 1.5); hold on;
plot(t_control, y_PI, 'b-', 'LineWidth', 1.5);
plot(t_control, y_PD, 'g-', 'LineWidth', 1.5);
plot(t_control, y_PID, 'm-', 'LineWidth', 1.5);
plot([0, max(t_control)], [1, 1], 'k--', 'LineWidth', 1);
grid on;
title('Closed-Loop Controller Comparison', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Time (seconds)', 'FontSize', 12);
ylabel('Output (Normalized)', 'FontSize', 12);
legend('P Controller', 'PI Controller', 'PD Controller', 'PID Controller', 'Setpoint', 'Location', 'best');
hold off;
saveas(gcf, 'controller_comparison.png');

fprintf('Controller Performance:\n');
fprintf('  P Controller - Steady-State Value: %.4f (Offset: %.4f)\n', y_P(end), 1-y_P(end));
fprintf('  PI Controller - Steady-State Value: %.4f (Offset: %.4f)\n', y_PI(end), 1-y_PI(end));
fprintf('  PD Controller - Steady-State Value: %.4f (Offset: %.4f)\n', y_PD(end), 1-y_PD(end));
fprintf('  PID Controller - Steady-State Value: %.4f (Offset: %.4f)\n\n', y_PID(end), 1-y_PID(end));

%% 12. Tuned vs Untuned PI Controller (Figure 4.4)
% Untuned controller (poor guesses)
Kc_untuned = Kc_ZN * 0.3;
tau_I_untuned = tau_I_ZN * 0.5;
G_c_PI_untuned = tf([Kc_untuned * tau_I_untuned, Kc_untuned], [tau_I_untuned, 0]);
G_cl_PI_untuned = feedback(G_c_PI_untuned * G_p, 1);

figure(5);
[y_PI_tuned, t_tuned] = step(G_cl_PI, t_control);
[y_PI_untuned, ~] = step(G_cl_PI_untuned, t_control);

plot(t_control, y_PI_untuned, 'r--', 'LineWidth', 1.5); hold on;
plot(t_control, y_PI_tuned, 'b-', 'LineWidth', 2);
plot([0, max(t_control)], [1, 1], 'k--', 'LineWidth', 1);
grid on;
title('Tuned vs Untuned PI Controller', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Time (seconds)', 'FontSize', 12);
ylabel('Output (Normalized)', 'FontSize', 12);
legend('Untuned PI', 'Tuned PI (Ziegler-Nichols)', 'Setpoint', 'Location', 'best');
hold off;
saveas(gcf, 'tuned_vs_untuned.png');

%% 13. Disturbance Rejection Test (Figure 4.5)
% Simulate disturbance at t=50s
t_dist = 0:0.1:150;
disturbance_time = 50;
disturbance_magnitude = 0.1;  % 10% disturbance

% Create input: setpoint step at t=0, disturbance step at t=50
setpoint = ones(size(t_dist));
disturbance = zeros(size(t_dist));
disturbance(t_dist >= disturbance_time) = disturbance_magnitude;

% Closed-loop response to setpoint
[y_sp, ~] = lsim(G_cl_PI, setpoint, t_dist);

% Response to disturbance (load disturbance enters at process input)
% Transfer function from disturbance to output in closed loop
G_dist = feedback(G_p, G_c_PI);
[y_dist, ~] = lsim(G_dist, disturbance, t_dist);

% Total response
y_total = y_sp + y_dist;

figure(6);
plot(t_dist, setpoint, 'k--', 'LineWidth', 1.5); hold on;
plot(t_dist, y_total, 'b-', 'LineWidth', 2);
xline(disturbance_time, 'r--', 'Disturbance Applied', 'LineWidth', 1.5, 'LabelOrientation', 'horizontal');
grid on;
title('Disturbance Rejection Test (PI Controller)', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Time (seconds)', 'FontSize', 12);
ylabel('Output (Normalized)', 'FontSize', 12);
legend('Setpoint', 'Controlled Output', 'Location', 'best');
hold off;
saveas(gcf, 'disturbance_rejection.png');

fprintf('=== Analysis Complete ===\n');
fprintf('All figures have been saved as PNG files.\n');