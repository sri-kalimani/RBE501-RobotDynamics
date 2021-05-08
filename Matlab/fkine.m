function T = fkine(S,M,q)
% FKINE  Returns forward kinematics using product of exponentials formula
%   Inputs: S - 6xn screw axes 
%           M - homogeneous transformatino matrix of home position
%           q - joint variables
%
%   Output: T - homogenous transformation matrix of end effector

    [m,n] = size(S);
    T = eye(4);
    for i = 1:n
        T = T * twist2ht(S(:,i),q(i));
    end
    T = T * M;
end