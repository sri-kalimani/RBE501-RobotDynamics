function T = twist2ht(S,theta)
% TWIST2HT Converts twist to homogenous transformation matrix
% Inputs: S - 6xn matrix representing the screw axes of all joints
%         theta - angle of rotation
% Output: T - Homogenous transformation matrix of all joints

    omega = S(1:3);
    w_skew = [0, -omega(3), omega(2); omega(3), 0, -omega(1); -omega(2), omega(1), 0];
    v = S(4:end);
    
    if abs(norm(omega)-1) < 0.01
        T = [axisangle2rot(omega,theta)  (eye(3)*theta + (1-cos(theta))*w_skew + (theta-sin(theta))*w_skew^2)*v; [0,0,0] 1];

    else
        T  = [eye(3) v*theta; [0 0 0] 1];
    
    end

end