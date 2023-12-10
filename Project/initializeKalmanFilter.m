function [x_hat, P] = initializeKalmanFilter()

    % State vector definition for target tracking
    % x_hat = [x; y; z; vx; vy; vz]
    % initial_target_position = @(t) [t; t ; t];
    x_hat = [0; 0; 0; 0; 0; 0];
    % Initial estimate of the state covariance matrix
    P = diag([1, 1, 1, 0.1, 0.1, 0.1]);

end
