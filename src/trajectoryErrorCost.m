function cost = trajectoryErrorCost(K_vec, A, B, Q, R, m, g, l, sigma, tspan, z0, zd, ud)
    K = reshape(K_vec, size(Q));
    
    % Define UAV trajectory function
    scale = rand(3, 1) * 5;
    uav_trajectory = @(t, z) [scale(1)*sin(3*t); -scale(2)*cos(2*t); scale(3)*cos(5*t); 0; 0; 0; 0; 0; 0; 0; 0; 0];

    % Update the drone equations to include the UAV trajectory
    dz = @(t, z, u) A*z + B*u + uav_trajectory(t, z);

    % Initialize initial conditions for the UAV trajectory
    z0 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];

    % Solve the system using the updated drone equations
    [t, z] = ode45(@(t, z) dz(t, z, u(z, zd, ud, K)), tspan, z0);

    % Compute the error and norm at each time step
    error_vector = z(:, 1:12) - repmat(zd', size(z, 1), 1);
    error_norm = norm(error_vector, 2);

    % Compute the cost (minimize the error norm)
    cost = error_norm;
end