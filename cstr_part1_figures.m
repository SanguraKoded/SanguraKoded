%% CSTR Analysis - PART 1: Figures 1 & 2
%% Pole-Zero Map and Open-Loop Step Response

clear all; close all; clc;

% Parameters
Y = 88;
Fi = (Y + 50) / 3600;
rho = 1000;
Cp = 4200;
V = 700 / 1000;
lambda_st = 2000 * 1000;

% Calculate transfer function parameters
tau = (rho * Cp * V) / (rho * Cp * Fi);
K = lambda_st / (rho * Cp * Fi);

fprintf('=== PART 1: Figures 1 & 2 ===\n');
fprintf('Time Constant (tau): %.2f s\n', tau);
fprintf('Process Gain (K): %.2f\n\n', K);

% Define transfer function
G_p = tf(K, [tau, 1]);

%% FIGURE 1: Pole-Zero Map
figure(1);
pzmap(G_p);
grid on;
title('Pole-Zero Map of CSTR Open-Loop Transfer Function', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Real Axis', 'FontSize', 12);
ylabel('Imaginary Axis', 'FontSize', 12);

poles_Gp = pole(G_p);
hold on;
plot(poles_Gp, 0, 'rx', 'MarkerSize', 15, 'LineWidth', 2);
text(poles_Gp - 0.001, 0.0005, sprintf('  Pole = %.6f', poles_Gp), 'FontSize', 10);
hold off;

fprintf('Figure 1 Generated: Pole-Zero Map\n');
fprintf('  Pole at s = %.6f\n\n', poles_Gp);

%% FIGURE 2: Open-Loop Step Response
step_mag = Y;
figure(2);
t_step = 0:1:5*tau;
[y_step, ~] = step(step_mag * G_p, t_step);
plot(t_step, y_step, 'b-', 'LineWidth', 2);
grid on;
title('Open-Loop Step Response', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Time (seconds)', 'FontSize', 12);
ylabel('Output Temperature (°C)', 'FontSize', 12);

ss_value = step_mag * K;
hold on;
plot([0, max(t_step)], [ss_value, ss_value], 'r--', 'LineWidth', 1.5);
legend('Step Response', sprintf('Steady State = %.2f', ss_value), 'Location', 'northeast');
hold off;

fprintf('Figure 2 Generated: Open-Loop Step Response\n');
fprintf('  Steady State: %.2f °C\n', ss_value);
fprintf('  Rise Time: %.2f s\n', 2.2*tau);
fprintf('  Settling Time: %.2f s\n\n', 4*tau);

fprintf('=== RIGHT-CLICK EACH FIGURE TO SAVE ===\n');