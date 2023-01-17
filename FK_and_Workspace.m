%% Forward Kinematics and Workspace
% For a 5 DoF Lynxmotion arm: http://www.lynxmotion.com/p-1179-lynxmotion-lss-5-dof-robotic-arm-kit.aspx
% Author: Tunwu Li

%%
clear
clc
close all

%% Initialization
% Radian system
syms d_1 L_1 L_2 L_3 t_1 t_2 t_3 t_4 psi

%% Homogeneous Matrix (SDH)
% Degreee system
% T_01
T_01 = Distal_val(0, 90, d_1, t_1);

R_01 = T_01(1:3, 1:3);
P_01 = T_01(1:3, 4);

% T_12
T_12 = Distal_val(L_1, 0, 0, t_2);

R_12 = T_12(1:3, 1:3);
P_12 = T_12(1:3, 4);

% T_23
T_23 = Distal_val(L_2, 0, 0, t_3);

R_23 = T_23(1:3, 1:3);
P_23 = T_23(1:3, 4);

% T_34
T_34 = Distal_val(0, 90, 0, t_4);

R_34 = T_34(1:3, 1:3);
P_34 = T_34(1:3, 4);


% T_45
T_45 = Distal_val(0, 0, L_3, 0);

R_45 = T_45(1:3, 1:3);
P_45 = T_45(1:3, 4);

% Compound transformation
% T_05
T_05 = T_01 * T_12 * T_23 * T_34 * T_45;

% Cartesian coordinates of the end-effector
X = simplify(T_05(1, 4))
Y = simplify(T_05(2, 4))
Z = simplify(T_05(3, 4))

%% Workspace
% Degree system
t_1 = 0 : 3 : 359;
t_2 = 0 : 1.5 : 179.5;
t_3 = 0 : 3 : 359;
t_4 = 0 : 3 : 359;
t_5 = 0 : 3 : 359;

d_1 = 100; % 100 mm
L_1 = 300;
L_2 = 400;
L_3 = 150;

%% 3D reachable workspace
x_work = zeros(360, 360); % reserving space for the variables, because
y_work = zeros(360, 360); % otherwise they would be created later within a loop.
z_work = zeros(360, 360);

for i = 1 : 120	% for theta1
    for j = 1 : 120   % for theta2
        for k = 1 : 120 % for theta3
            for q = 1 : 120 % for theta4

                x_work(i, j) = cosd(t_1(i))*(L_2*cosd(t_2(j) + t_3(k)) + L_1*cosd(t_2(j)) + L_3*sind(t_2(j) + t_3(k) + t_4(q)));
                y_work(i, j) = sind(t_1(i))*(L_2*cosd(t_2(j) + t_3(k)) + L_1*cosd(t_2(j)) + L_3*sind(t_2(j) + t_3(k) + t_4(q)));
                z_work(i, j) = d_1 + L_2*sind(t_2(j) + t_3(k)) + L_1*sind(t_2(j)) - L_3*cosd(t_2(j) + t_3(k) + t_4(q));
                
            end
        end
    end
end

figure (1)
plot3(x_work, y_work, z_work, '.')
title('3D')
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal

figure (2)
plot(x_work, y_work)
title('XY Plane')
xlabel('X'); ylabel('Y');

figure (3)
plot(x_work, z_work)
title('XZ Plane')
xlabel('X'); ylabel('Z');

figure (4)
plot(y_work, z_work)
title('YZ Plane')
xlabel('Y'); ylabel('Z');
