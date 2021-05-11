function T = fkine(S,M,q)
% FKINE  Returns forward kinematics using product of exponentials formula
%   Inputs: S - 6xn screw axes 
%           M - homogeneous transformatino matrix of home position
%           q - joint variables
%
%   Output: T - homogenous transformation matrix of end effector

    [m,n] = size(S);
    T_c = eye(4);
    for i = 1:n
        T_c = T_c * twist2ht(S(:,i),q(i));
    end
    T = T_c * M;
end