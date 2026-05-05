clc
clear all
close all

% Inverse Kinematics Implementation in MATLAB

% Link Lengths
L1 = 20; % Length of the first link (Z-offset of base)
L2 = 135; % Length of the second link
L3 = 165; % Length of the third link
L4 = 80;  % Length of the fourth link (end-effector)

% Desired End-Effector Position
x = 100;  % X-coordinate of end-effector
y = 50;   % Y-coordinate of end-effector
z = 10;    % Z-coordinate of end-effector

D = sqrt(x^2 + y^2); % Horizontal distance from base to end-effector

% First Joint Angle (Theta1)
Theta1 = atan2(y, x); % Angle in radians
Theta1_deg = rad2deg(Theta1); % Convert to degrees

% Effective Length (Projection on ZX plane)
d = D - L4; % Effective horizontal distance
z_offset = z - L1; % Effective vertical distance

% Distance to the target point in ZX plane
R = sqrt(d^2 + z_offset^2);

% Alpha1 Calculation
cos_alpha1 = d / R;
alpha1 = acos(cos_alpha1);
alpha1_deg = rad2deg(alpha1);

% Alpha2 Calculation
cos_alpha2 = (L2^2 + R^2 - L3^2) / (2 * L2 * R);
alpha2 = acos(cos_alpha2);
alpha2_deg = rad2deg(alpha2);

% Theta3 Calculation
cos_theta3 = (L2^2 + L3^2 - R^2) / (2 * L2 * L3);
Theta3 = acos(cos_theta3);
Theta3_deg = rad2deg(Theta3);

if (z <= L1)
    % Theta2 Calculation
    Theta2 = alpha2 - alpha1;
    Theta2_deg = rad2deg(Theta2);
    Theta4 = 180 - ((180 - (alpha2_deg + Theta3_deg)) + alpha1_deg);
else
    Theta2 = alpha2 + alpha1;
    Theta2_deg = rad2deg(Theta2);
    Theta4 = 180 - ((180 - (alpha2_deg + Theta3_deg)) - alpha1_deg); 
end

% Display Results
fprintf('Theta1 (degrees): %.2f\n', Theta1_deg);
fprintf('Theta2 (degrees): %.2f\n', Theta2_deg);
fprintf('Theta3 (degrees): %.2f\n', Theta3_deg);
fprintf('Theta4 (degrees): %.2f\n', Theta4);

