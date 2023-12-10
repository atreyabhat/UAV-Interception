function [x_hat, P] = kalmanFilterUpdate(x_hat, P, z_meas, A, H, Q, R)

    % Prediction Step (Time Update)
    x_hat_minus = A * x_hat;
    P_minus = A * P * A' + Q;

    % Update Step (Measurement Update)
    K = P_minus * H' / (H * P_minus * H' + R); % Kalman gain
    x_hat = x_hat_minus + K * (z_meas - H * x_hat_minus);
    P = (eye(6) - K * H) * P_minus;

end
