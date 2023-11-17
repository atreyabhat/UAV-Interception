function [x_hat, P, estimated_state] = kalman_filter(A, H, Q, R, x_hat, P, noisy_measurement)
    % Prediction step
    x_hat_minus = A * x_hat;
    P_minus = A * P * A' + Q;

    % Update step
    K = P_minus * H' / (H * P_minus * H' + R);
    x_hat = x_hat_minus + K * (noisy_measurement - H * x_hat_minus);
    P = (eye(size(P)) - K * H) * P_minus;

    % Estimated state
    estimated_state = x_hat;
end
