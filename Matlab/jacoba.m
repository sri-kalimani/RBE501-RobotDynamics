function J_a = jacoba(S,M,q)    
% Your code here
 transform = fkine(S,M,q);
    p = transform(1:3,4);
    skewP = [0   ,  -p(3),  p(2);
            p(3) ,      0, -p(1);
            -p(2),   p(1),     0;];
    jacobian = jacob0(S,q);
    jW = jacobian(1:3,:);
    jV = jacobian(4:6,:);

    J_a = -(skewP*jW) + jV;
end