function J = jacob0(S,q) 
% JACOB0 Returns jacobian matrix of all joints
% Inputs: S - 6xn matrix representing the screw axes of all joints
%         q - 1x3 matrix representing the currrent joint values
% Output: J - 6xn Jacobian matrix
    
    J = zeros(6,length(q));
    J(:,1) = S(:,1);      
    T = eye(4);
    
    for i = 2:length(q) 
        
        V = S(:,i-1);
        omega = q(i-1);
        T = T*twist2ht(V,omega);
        
        V = S(:,i);
        J(:,i)= adjoint(V,T);
    end
end