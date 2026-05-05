clc
clear all
close all

%% Defining Variables and Inputs From Users
syms theta1 theta2 theta3 theta4 theta5 theta6 

% DH Table Preparation
DH_table = sym(zeros(5, 4)); 

% Getting parameters from user
for i = 1:5
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

%% Creating Homogeneous Transformation Matrices

T_overall = eye(4);
T_matrices = cell(5, 1);
for i = 1:5
    T_rot_x = [1, 0, 0, 0; 
               0, cos(DH_table(i,1)), -sin(DH_table(i,1)), 0; 
               0, sin(DH_table(i,1)), cos(DH_table(i,1)), 0; 
               0, 0, 0, 1];

    T_trans_x = [1, 0, 0, DH_table(i,2); 
                 0, 1, 0, 0; 
                 0, 0, 1, 0; 
                 0, 0, 0, 1];
    
    T_trans_z = [1, 0, 0, 0; 
                 0, 1, 0, 0; 
                 0, 0, 1, DH_table(i,3); 
                 0, 0, 0, 1];
    
    T_rot_z = [cos(DH_table(i,4)), -sin(DH_table(i,4)), 0, 0; 
               sin(DH_table(i,4)), cos(DH_table(i,4)), 0, 0; 
               0, 0, 1, 0; 
               0, 0, 0, 1];
    
    T_transformation = simplify(T_rot_z * T_trans_z * T_trans_x * T_rot_x); 
    
    T_overall = simplify(T_overall * T_transformation); 
    
    T_matrices{i} = T_transformation;
    
    fprintf('Transformation Matrix For Joint %d:\n', i);
    disp(vpa(T_transformation, 4));
    
    % Extract and display the rotation matrix (top-left 3x3 block)
    R_matrix = T_transformation(1:3, 1:3);
    fprintf('Rotation Matrix For Joint %d:\n', i);
    disp(vpa(R_matrix, 4)); % Display the rotation matrix with numeric precision (if possible)
end

disp('Homogeneous Transformation Matrices (Overall):');
disp(vpa(T_overall, 4)); % Display the overall transformation matrix with numeric precision
