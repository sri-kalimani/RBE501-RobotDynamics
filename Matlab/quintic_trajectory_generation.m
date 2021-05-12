n = 6;
q0 = [-20 40 20 -10 55 -70];
qf = [0 0 0 0 0 0];
time = 1001; %Total duration of the movement in millis
time_step = 50; %Number of millis between each iteration in control loop

overall_matrix = [];
for ii = 1:n %Iteratre over each joint
    coeffs = quinticpoly(0, time, q0(ii), qf(ii), 0, 0, 0, 0);
    num_iterations = time / time_step;
    trajectory = [];
    for jj = 1:num_iterations
        t = jj * time_step;
        q_t = round(coeffs(1) + coeffs(2)*t + coeffs(3)*t^2 + ...
        coeffs(4)*t^3 + coeffs(5)*t^4 +  + coeffs(6)*t^5);
        trajectory = [trajectory q_t];
    end
    %Append to the overall matrix
    overall_matrix = [overall_matrix; trajectory];
end

%Export results to CSV
writematrix(overall_matrix, 'move7.csv')