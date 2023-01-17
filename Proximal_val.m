%% Forward Kinematics Function for Proximal Approach
% Author: Tunwu Li

%% 
function Proximal_val=Proximal_val(a, alpha, d, theta)
% The value of the general expression for a homogeneous transformation matrix
% Degree system
% a: ; alpha: ; d: ; theta: 
% 编写于 2022.11.06

Proximal_val = [cosd(theta)             -sind(theta) 0  a;
                sind(theta)*cosd(alpha)  cosd(theta)*cosd(alpha) -sind(alpha) -sind(alpha)*d;
                sind(theta)*sind(alpha)  cosd(theta)*sind(alpha)  cosd(alpha)  cosd(alpha)*d;
                zeros(1, 3) 1];
end