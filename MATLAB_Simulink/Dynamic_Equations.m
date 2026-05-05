%% MTE 491 Robotic Design Project 3
% Furkan Güntaş - Orhan Köseoğlu - Emircan Yücel

%% Cleaning All Variables, Graphs, etc.
clear all
close all
clc

%% Defining Variables, Creating User Interface, Taking Symbolic Inputs From Users
syms theta1 theta2 theta3 theta4 theta5 theta6 
syms L1 L2 L3 L4 L5 L6 
syms d1 d2 d3 d4 d5 d6 
syms alpha1 alpha2 alpha3 alpha4 alpha5 alpha6 
syms g real

% Taking number of joints from the users.
DOF = input('Please enter the joint number of the robot (should be between 1-6): ');

% DOF validation
if DOF < 1 || DOF > 6
    error('Error: The degrees of freedom must be between 1 and 6.');
end
disp(['Degrees of freedom of your system is: ', num2str(DOF)]);

% DH Table Preparation
DH_table = sym(zeros(DOF, 4)); 
joint_type = cell(DOF, 1); 

% Getting parameters from user
for i = 1:DOF
    fprintf('Is joint %d Revolute or Prismatic: ', i);
    joint_type{i} = input('(Enter "R" or "P"): ', 's');
    while ~ismember(joint_type{i}, {'R', 'P'})
        disp('Invalid input. Please enter "R" or "P".');
        joint_type{i} = input('Is this joint Revolute or Prismatic? (Enter "R" or "P"): ', 's');
    end
    
    fprintf('Please enter the symbolic parameters for joint %d:\n', i);
    twist_angle = input('Twist Angle (alpha): ', 's');
    link_length = input('Link Length (L): ', 's');
    offset = input('Offset (d): ', 's');
    joint_angle = input('Joint Angle (theta): ', 's');
    
    DH_table(i, :) = [sym(twist_angle), sym(link_length), sym(offset), sym(joint_angle)];
end

disp('The DH table of your system according to your entered parameters is:');
disp(' '); % Add a blank line after each row

disp(array2table(DH_table, 'VariableNames', {'Twist Angle', 'Link Length (a)', 'Offset (d)', 'Joint Angle'}));

%% Creating Homogeneous Transformation Matrices for each joint
T_overall = eye(4);
T_matrices = cell(DOF, 1);
for i = 1:DOF
    T_rot_x = [1, 0, 0, 0; 0, cos(DH_table(i,1)), -sin(DH_table(i,1)), 0; 0, sin(DH_table(i,1)), cos(DH_table(i,1)), 0; 0, 0, 0, 1];
    T_trans_x = [1, 0, 0, DH_table(i,2); 0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
    T_trans_z = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, DH_table(i,3); 0, 0, 0, 1];
    T_rot_z = [cos(DH_table(i,4)), -sin(DH_table(i,4)), 0, 0; sin(DH_table(i,4)), cos(DH_table(i,4)), 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
    T_transformation = T_rot_z * T_trans_z * T_trans_x * T_rot_x;
    T_overall = T_overall * T_transformation;
    T_matrices{i} = T_overall;
end

%% Obtaining Jacobian Matrix
Jacobian = sym(zeros(6, DOF));
Z_prev = [0; 0; 1];
Origin_prev = [0; 0; 0];
Position_vector = T_matrices{end}(1:3, 4);
for i = 1:DOF
    T_current = T_matrices{i};
    Z_current = T_current(1:3, 3);
    Origin_current = T_current(1:3, 4);
    if joint_type{i} == 'R'
        Jacobian(1:3, i) = cross(Z_prev, (Position_vector - Origin_prev));
        Jacobian(4:6, i) = Z_prev;
    elseif joint_type{i} == 'P'
        Jacobian(1:3, i) = Z_prev;
        Jacobian(4:6, i) = [0; 0; 0];
    end
    Z_prev = Z_current;
    Origin_prev = Origin_current;
end
disp('Jacobian Matrix:');
disp(' '); % Add a blank line after each row
disp(Jacobian);



%% User Input for External Forces and Torques
disp('Enter external forces acting on the end-effector:');
disp(' '); % Add a blank line after each row
syms Fx Fy Fz Tx Ty Tz real;
Fx = input('Enter force in X-direction (Fx): ');
Fy = input('Enter force in Y-direction (Fy): ');
Fz = input('Enter force in Z-direction (Fz): ');

Force_ext = [Fx; Fy; Fz; 0; 0; 0]; % 0'lar dimension icin
disp(' '); % Add a blank line after each row

% External Joint Torques Calculation
External_joint_torques = simplify(Jacobian.' * Force_ext);

disp('External Joint Torque Contribution (Tau):');
disp(' '); % Add a blank line after each row
disp(External_joint_torques);

% Gravity Torque Calculation for Joint Space

% Initialize symbolic variables
syms g real; % Gravitational acceleration
syms m [DOF 1] real; % Mass of each link

% Initialize gravity torque vector
Gravity_Torque = sym(zeros(DOF, 1)); % To store the torque for each joint

% Iterate over each joint to calculate its gravity torque
for i = 1:DOF
    for j = i:DOF
        % Add the contribution of the j-th link's mass to the i-th joint
        Gravity_Torque(i) = Gravity_Torque(i) + m(j) * g * Jacobian(3, j); % Use z-component (3rd row) of Partial Jacobian
    end
end

%% Mass Matrix Calculation 
disp('Calculating Cartesian Mass Matrix...');

% Define symbolic variables
syms m [DOF 1] real; % Mass of each link
for i = 1:DOF
    m(i) = input(['Enter mass of link ', num2str(i), ': ']);
end

% Joint-space mass matrix
Mass_Matrix = diag(m); % DOF x DOF diagonal mass matrix

% Inverse and Transpose of Jacobian
Jacobian_Inv = simplify(inv(Jacobian(1:DOF, 1:DOF))); % Inverse of reduced Jacobian
Jacobian_Inv_T = simplify(Jacobian_Inv.'); % Transpose of the inverse Jacobian

% Cartesian-space mass matrix calculation
Cartesian_Mass_Matrix = simplify(Jacobian_Inv_T * Mass_Matrix * Jacobian_Inv);

disp('Cartesian Mass Matrix:');
disp(' '); % Add a blank line after each row
disp(Cartesian_Mass_Matrix);

%% Coriolis and Centrifugal Matrix Calculation

% Symbolic variables
syms q [DOF 1] real; % Generalized joint positions
syms dq [DOF 1] real; % Generalized joint velocities
syms M [DOF DOF] real; % Inertia (mass) matrix

% Initialize the Coriolis matrix (C)
C = sym(zeros(DOF, DOF)); 

% Calculate Christoffel symbols of the first kind (c_ijk)
for i = 1:DOF
    for j = 1:DOF
        for k = 1:DOF
            c_ijk = (1/2) * (diff(M(i,j), q(k)) + diff(M(i,k), q(j)) - diff(M(j,k), q(i)));
            C(i,j) = C(i,j) + c_ijk * dq(k); % Summing over k
        end
    end
end

% Simplify the resulting C matrix
C = simplify(C);

%% Joint Accelerations Calculation (Simplified for Joint Space)
disp('Calculating Joint Accelerations...');

% Compute joint accelerations in joint space
Theta_ddot = simplify(inv(Cartesian_Mass_Matrix) * (External_joint_torques - C - Gravity_Torque));

disp('Joint Accelerations (Theta_ddot):');
for i = 1:DOF
    fprintf('Joint %d Acceleration: ', i);
    disp(Theta_ddot(i));
end

%% Asking for User Inputs and Constructing Numerical D-H Table
disp('Enter numerical values for the DH Table:');
DH_table = zeros(DOF, 4); % Initialize the DH table
joint_type = cell(DOF, 1); % Initialize joint type storage

for i = 1:DOF
    fprintf('For Joint %d:\n', i);
    twist_angle = input('Twist Angle (alpha) in degrees: ');
    link_length = input('Link Length (L): ');
    offset = input('Offset (d): ');
    joint_angle = input('Joint Angle (theta) in degrees: ');
    DH_table(i, :) = [deg2rad(twist_angle), link_length, offset, deg2rad(joint_angle)];
    
    fprintf('Is joint %d Revolute or Prismatic? (Enter "R" or "P"): ', i);
    joint_type{i} = input('', 's');
    while ~ismember(joint_type{i}, {'R', 'P'})
        disp('Invalid input. Please enter "R" or "P".');
        joint_type{i} = input('Enter "R" for Revolute or "P" for Prismatic: ', 's');
    end
end

disp('Enter numerical values for the masses of the links:');
masses = zeros(DOF, 1);
for i = 1:DOF
    masses(i) = input(['Mass of Link ', num2str(i), ' in kg: ']);
end

disp('Enter numerical values for external forces acting on the end-effector:');
Fx = input('Force in X-direction (Fx) in N: ');
Fy = input('Force in Y-direction (Fy) in N: ');
Fz = input('Force in Z-direction (Fz) in N: ');

Force_ext = [Fx; Fy; Fz; 0; 0; 0]; % External forces vector

%% Substituting Numerical Values into Transformation Matrices and Jacobian

T_matrices = cell(DOF, 1); % Store transformation matrices
T_overall = eye(4); % Initial transformation matrix

% Compute transformation matrices
for i = 1:DOF
    alpha = DH_table(i, 1);
    a = DH_table(i, 2);
    d = DH_table(i, 3);
    theta = DH_table(i, 4);

    % Compute individual transformation matrices
    T_rot_x = [1, 0, 0, 0; 0, cos(alpha), -sin(alpha), 0; 0, sin(alpha), cos(alpha), 0; 0, 0, 0, 1];
    T_trans_x = [1, 0, 0, a; 0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
    T_trans_z = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, d; 0, 0, 0, 1];
    T_rot_z = [cos(theta), -sin(theta), 0, 0; sin(theta), cos(theta), 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];

    % Compute the overall transformation matrix
    T_current = T_rot_x * T_trans_x * T_trans_z * T_rot_z;
    T_overall = T_overall * T_current;

    % Store the current transformation matrix
    T_matrices{i} = T_overall;
end

% Compute the Jacobian matrix numerically
Jacobian_num = zeros(6, DOF);
Z_prev = [0; 0; 1];
Origin_prev = [0; 0; 0];
Position_vector = T_matrices{end}(1:3, 4);

for i = 1:DOF
    T_current = T_matrices{i};
    Z_current = T_current(1:3, 3);
    Origin_current = T_current(1:3, 4);

    if joint_type{i} == 'R'
        Jacobian_num(1:3, i) = cross(Z_prev, (Position_vector - Origin_prev));
        Jacobian_num(4:6, i) = Z_prev;
    elseif joint_type{i} == 'P'
        Jacobian_num(1:3, i) = Z_prev;
        Jacobian_num(4:6, i) = [0; 0; 0];
    end

    Z_prev = Z_current;
    Origin_prev = Origin_current;
end

% Gravity Torque Calculation
g = 9.81; % Gravitational acceleration
Gravity_Torque_num = zeros(DOF, 1);
for i = 1:DOF
    for j = i:DOF
        Gravity_Torque_num(i) = Gravity_Torque_num(i) + masses(j) * g * Jacobian_num(3, j);
    end
end

% External Joint Torques Calculation
External_joint_torques_num = Jacobian_num.' * Force_ext;

% Mass Matrix Calculation
Mass_Matrix_num = diag(masses);

% Coriolis and Centrifugal Matrix Calculation
C_num = zeros(DOF, DOF);

for i = 1:DOF
    for j = 1:DOF
        for k = 1:DOF
            c_ijk_num = 0.5 * (diff(Mass_Matrix_num(i, j), q(k)) + ...
                               diff(Mass_Matrix_num(i, k), q(j)) - ...
                               diff(Mass_Matrix_num(j, k), q(i)));
            C_num(i, j) = C_num(i, j) + c_ijk_num * dq(k);
        end
    end
end

% Joint Accelerations Calculation with Coriolis
disp('Calculating Joint Accelerations...');

Theta_ddot_num = inv(Mass_Matrix_num) * (External_joint_torques_num - C_num - Gravity_Torque_num);

disp('Joint Accelerations (Theta_ddot):');
for i = 1:DOF
    fprintf('Joint %d Acceleration: %.4f rad/s^2\n', i, Theta_ddot_num(i));
end