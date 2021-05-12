n = 6;
q0 = [36 -12 23 0 0 70];
qf = [32 -30 40 0 0 70];
time = 1500; %Total duration of the movement in millis
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
writematrix(overall_matrix, 'move3.csv')