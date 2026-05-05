clc;
close all;
clear all;

% Get the initial and final angles along with durations for each joint
num_joints = 4; % Number of joints

theta_0 = zeros(1, num_joints); % Initial angles
theta_f = zeros(1, num_joints); % Final angles
T = zeros(1, num_joints); % Durations

for i = 1:num_joints
    fprintf('Joint %d:\n', i);
    theta_0(i) = input(sprintf('  Enter the initial angle (theta_0) for Joint %d: ', i));
    theta_f(i) = input(sprintf('  Enter the final angle (theta_f) for Joint %d: ', i));
    T(i) = input(sprintf('  Enter the total duration (T) for Joint %d: ', i));
end

% Create a single figure for all joints
figure;

% Perform calculations for each joint
for j = 1:num_joints
    % Time powers
    T2 = T(j)^2;
    T3 = T(j)^3;
    T4 = T(j)^4;
    T5 = T(j)^5;

    % Assume initial velocity and acceleration are zero
    dtheta_0 = 0; % Initial velocity
    ddtheta_0 = 0; % Initial acceleration
    dtheta_f = 0; % Final velocity
    ddtheta_f = 0; % Final acceleration

    % Coefficients calculation using boundary conditions
    a0 = theta_0(j);
    a1 = dtheta_0;
    a2 = ddtheta_0 / 2;
    a3 = (20*theta_f(j) - 20*theta_0(j) - (8*dtheta_f + 12*dtheta_0)*T(j) - (3*ddtheta_0 - ddtheta_f)*T2) / (2*T3);
    a4 = (30*theta_0(j) - 30*theta_f(j) + (14*dtheta_f + 16*dtheta_0)*T(j) + (3*ddtheta_0 - 2*ddtheta_f)*T2) / (2*T4);
    a5 = (12*theta_f(j) - 12*theta_0(j) - (6*dtheta_f + 6*dtheta_0)*T(j) - (ddtheta_0 - ddtheta_f)*T2) / (2*T5);

    %% Fifth Order Calculation
    t = 0:0.001:T(j);

    theta = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5;
    theta_dot = a1 + 2*a2*t + 3*a3*t.^2 + 4*a4*t.^3 + 5*a5*t.^4;
    theta_double_dot = 2*a2 + 2*3*a3*t + 3*4*a4*t.^2 + 4*5*a5*t.^3;

    % Plot position
    subplot(3, num_joints, j);
    plot(t, theta, 'LineWidth', 2);
    grid on;
    title(sprintf('Joint %d Position', j));
    xlabel('Time (Second)');
    ylabel('Position (Degrees)');

    % Plot velocity
    subplot(3, num_joints, num_joints + j);
    plot(t, theta_dot, 'LineWidth', 2);
    grid on;
    title(sprintf('Joint %d Velocity', j));
    xlabel('Time (Second)');
    ylabel('Velocity (Degrees/sec)');

    % Plot acceleration
    subplot(3, num_joints, 2*num_joints + j);
    plot(t, theta_double_dot, 'LineWidth', 2);
    grid on;
    title(sprintf('Joint %d Acceleration', j));
    xlabel('Time (Second)');
    ylabel('Acceleration (Degrees/sec^2)');
end