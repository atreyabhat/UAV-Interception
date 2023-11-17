
m = 0.5; % kg
g = 9.81; % m/s/s
l = 0.2; %m
I = [1.24, 0,     0;
     0,   1.24,   0;
     0,   0,   2.48];
I11 = 1.24;
I22 = I11;
I33 = 2.48;
minF = 0.0;
maxF = 3.0;
sigma = 0.01;

% Define A matrix
A11 = zeros(6, 6);
A12 = eye(6);
A22 = zeros(6,6);
A21 = [0, 0, 0, 0, g, 0;
       0, 0, 0, -g, 0, 0;
       0, 0, 0, 0, 0, 0;
       0, 0, 0, 0, 0, 0;
       0, 0, 0, 0, 0, 0;
       0, 0, 0, 0, 0, 0];

A = [A11, A12; A21, A22];

% Define B matrix
B1 = zeros(6, 4);
B2 = [0, 0, 0, 0;
      0, 0, 0, 0; 
      1/m, 1/m,1/m, 1/m; 
      0, l/I11,0, -l/I11; 
      -l/I22,0, l/I22,0; 
      sigma/I33, -sigma/I33, sigma/I33, -sigma/I33];


B = [B1; B2];
%rankc = rank(ctrb(A, B));


dt = 0.01; 
t = 2000;  
time = (0:dt:(t-1)*dt);

x = zeros(12, t);
u = zeros(4, t);

% Initial state
x(:, 1) = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];  % assuming initial position and velocities are zero

%LQR
Q = diag([20, 20, 20, 1, 1, 1, 1, 1, 1, 1, 1, 1]);  % State cost
R = eye(4);  % Control cost
[K, ~, ~] = lqr(A, B, Q, R);


% Initialize Kalman filter variables, Constant velocity model
x_hat = zeros(9, 1);
P_kf = eye(9);
Q_kf = diag([1, 1, 1, 1, 1, 1, 1, 1, 1]); 
R_kf = (0.1)^2 * eye(6);
A_kf = [1 0 0 dt 0 0 0 0 0;
     0 1 0 0 dt 0 0 0 0;
     0 0 1 0 0 dt 0 0 0;
     0 0 0 1 0 0 0 0 0;
     0 0 0 0 1 0 0 0 0;
     0 0 0 0 0 1 0 0 0;
     0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 0 1];
H_kf = eye(6, 9);



uav_trajectory = zeros(6, t);
uav_trajectory_estimated = zeros(6,t);
u = zeros(4, t);

% Generate elliptical trajectory in 3D
a = 20; % semi-major axis
b = 10; % semi-minor axis
z_height = 10; % height above the xy-plane

for k = 1:(t - 1)
    theta = linspace(0, 2 * pi, t);
    x_ellipse = a * cos(theta(k));
    y_ellipse = b * sin(theta(k));
    
    uav_state = [x_ellipse; y_ellipse; z_height; 0; 0; 0];
    uav_trajectory(:, k) = uav_state;
end

for k = 1:(t - 1)
    
    [x_hat, P_kf, estimated_state_kf] = kalman_filter(A_kf, H_kf, Q_kf, R_kf, x_hat, P_kf, uav_trajectory(:, k));
    
    desired_state =  estimated_state_kf(1:6);
    %append to array for plotting later
    uav_trajectory_estimated(:,k) = desired_state;

    % LQR control, Pass the traj estimated from KF
    u(:, k) = -K * (x(:, k) - [desired_state; zeros(6, 1)]);

    x_dot = A * x(:, k) + B * u(:, k);
    x(:, k + 1) = x(:, k) + x_dot * dt;

    % Display control inputs at each iteration
    fprintf('Iteration: %d, Control Inputs (u): [%f, %f, %f, %f]\n', k, u(1, k), u(2, k), u(3, k), u(4, k));
end


figure;
plot3(x(1, :), x(2, :), x(3, :), 'LineWidth', 1.5);
hold on;
plot3(uav_trajectory_estimated(1, :), uav_trajectory_estimated(2, :), uav_trajectory_estimated(3, :), 'LineWidth', 1.5);
title('Drone Movement in 3D');
legend('Actual Trajectory','UAV Trajectory (KF)');
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
axis equal;
grid on;

axisLimits = [-30, 30, -30, 30, 0, 20]; 

axis(axisLimits);
axis equal;
grid on;



% % Animate the plot
% for k = 1:(t - 1)
%     % Update the actual trajectory plot
%     plot3(x(1, 1:k), x(2, 1:k), x(3, 1:k), 'LineWidth', 1.5);
%     hold on;
%     plot3(uav_trajectory_estimated(1, 1:k), uav_trajectory_estimated(2, 1:k), uav_trajectory_estimated(3, 1:k), 'LineWidth', 1.5);
%     hold off;
% 
%     axis(axisLimits);
%     drawnow; % Refresh the plot
% end


