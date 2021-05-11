function J_b = jacobe(S,M,q)    
    % Your code here
    J_s = jacob0(S,q);
    J_b = adjoint(J_s,inv(fkine(S,M,q))); %space
end
