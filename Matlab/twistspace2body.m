function V_b = twistspace2body(V_s,T)
    % your code here
    p = T(1:3, 4:end);
    R = T(1:3, 1:3);
    p_skew = [0, -p(3), p(2); p(3), 0, -p(1); -p(2), p(1), 0];
    adj_inv = inv([R, zeros(3); p_skew*R, R]);
    V_b = adj_inv * V_s;
    
end