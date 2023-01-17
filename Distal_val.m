%% Forward Kinematics Function for Distal Approach
% Author: Tunwu Li

%% 
function Distal_val = Distal_val(a, alpha, d, t)
% A general expression for a homogeneous transformation matrix
% Degree system
% a alpha d theta 
% 编写于 2022.11.11

Distal_val = [cosd(t) -cosd(alpha)*sind(t)  sind(alpha)*sind(t) a*cosd(t);
              sind(t)  cosd(alpha)*cosd(t) -sind(alpha)*cosd(t) a*sind(t);
              0        sind(alpha)          cosd(alpha)         d;
              zeros(1, 3)  1];
end