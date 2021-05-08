function twist_inB = adjoint(V,T)
% ADJOINT  Returns adjoint matrix 
%   Inputs: V - twist 
%           T - homogeneous transformation matrix            
%
%   Output: twist_inB - homogenous transformation matrix of end effector
    R = T(1:3, 1:end-1);
    p = T(1:3, end);
    p_skew = [0, -p(3), p(2); p(3), 0, -p(1); -p(2), p(1), 0];
    twist_inB = [R, zeros(3,3); p_skew*R, R] * V;
end

