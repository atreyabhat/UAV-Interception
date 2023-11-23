clc; clear; close all;

%% Define the Parameters
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

%% Robot equations
dz = @(t, z, u) A*z + B*u;

u = @(z, zd, ud, K) ud + K*(zd - z);

%% Bug kinematics:

% a = 0.3; w = 10;
% db = @(t,b) [a*w*cos(t*w)*cos(t) - sin(t)*(a*sin(t*w) + 1);
%              cos(t)*(a*sin(t*w) + 1) + a*w*cos(t*w)*sin(t);
%              1];
scale = rand(3, 1)*5; % Generate random multipliers for x, y, and z axes
db = @(t, b) [scale(1)*sin(3*t); -scale(2)*cos(2*t); scale(3)*cos(5*t)];

%% Controller setup/cost

Q = diag([10, 10, 10, 1, 1, 1,1,1,1,1,1,1]);  % State cost
R = eye(4);  % Control cost
[K, ~, ~] = lqr(A, B, Q, R);

%% Initial Conditions
z0 = [0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
% u = [1,1,1,1] *m*g/4;
b0 = [1;1;2];

%% Final states
zf = [0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
uf = zeros(4, 1);


%% Drone Body
drone_body = [ 0.265,      0,     0, 1; ...
                    0, -0.265,     0, 1; ...
               -0.265,      0,     0, 1; ...
                    0,  0.265,     0, 1]';
                    % 0,      0,     0, 1; ...
                    % 0,      0, -0.15, 1]';


%% Problem parameters
epsilon = 0.1;
% %% Solve system from every initial state
% [t, z] = ode45( @(t, z) dz(t, z, u(z, zd, ud, K)), tspan, z0);

%% Phase I: Pursue
tspan_I = [0 10];
v0_I = [z0; b0];

options = odeset('Event', @(t,v) catchBug(t, v, epsilon),...
    'RelTol',1e-6);

[t_I, v_I, te, ve] = ode45( @(t, v) augmentedSystem(t, v, dz, db, u, K),...
    tspan_I, v0_I, options);

%% Init. 3D Fig.
fig1 = figure('pos',[0 200 800 800]);
h = gca;
view(3);
axis equal;
grid on;
xlim([-2 10]);
ylim([-2 10]);
zlim([0 12]);
xlabel('X[m]');
ylabel('Y[m]');
zlabel('Height[m]');

hold(gca, 'on');

wHb = [RPY2Rot(z0(7:9))' z0(1:3); 0 0 0 1];
% [Rot(also contains shear, reflection, local sacling), displacement; perspective ,global scaling]
drone_world = wHb * drone_body; % [4x4][4x4]
drone_atti = drone_world(1:3, :); 
    
fig1_ARM13 = plot3(gca, drone_atti(1,[1 3]), drone_atti(2,[1 3]), drone_atti(3,[1 3]), ...
        '-ro', 'MarkerSize', 5);
fig1_ARM24 = plot3(gca, drone_atti(1,[2 4]), drone_atti(2,[2 4]), drone_atti(3,[2 4]), ...
        '-bo', 'MarkerSize', 5);
fig1_shadow = plot3(gca,4, 4, 0,'xk','Linewidth',3);

quadrotorColor = [0, 0.4 0.7];
uavColor = [0.9, 0.3 0.1];
if(isempty(te))
    te = inf;
end
Q = linspace(0, 2*pi, 15)';
circle = epsilon*[cos(Q) sin(Q) cos(Q)];

epsilonNeigborhood = plot3(gca, 0, 0, 0, 'k', 'LineWidth', 1);
uav = plot3(1, 1, 2, 'o', 'Color', uavColor,...
    'MarkerFaceColor', uavColor, 'MarkerSize', 10,...
    'DisplayName','Bug');
uavTrace = plot3(1, 1, 2, '-', 'Color', uavColor);

quad = plot3(gca, 0, 0, 1, 's', 'Color', quadrotorColor,...
    'MarkerFaceColor', quadrotorColor, 'MarkerSize', 1,...
    'DisplayName','Robot');
quadTrace = plot3(gca, 0, 0, 1, '-', 'Color', quadrotorColor);

% hold(gca, 'off');
legend([uav, quad], 'Location', 'northeast');

%% Animate
t = [t_I];
z = [v_I(:,1:12)];
b = [v_I(:,[13, 14, 15])];

for k=1:length(t)
    % disp(z(k, 1:3))
    wHb = [RPY2Rot(z(k, 7:9))' z(k, 1:3)'; 0 0 0 1];
    drone_world = wHb * drone_body;
    drone_atti = drone_world(1:3, :);
    
	set(fig1_ARM13, ...
        'XData', drone_atti(1,[1 3]), ...
        'YData', drone_atti(2,[1 3]), ...
        'ZData', drone_atti(3,[1 3]));
    set(fig1_ARM24, ...
        'XData', drone_atti(1,[2 4]), ...
        'YData', drone_atti(2,[2 4]), ...
        'ZData', drone_atti(3,[2 4]));
	set(fig1_shadow, ...
		'XData', v_I(k,1), ...
		'YData', v_I(k,2),'ZData',0);

    set(uav,'XData', b(k,1),'YData', b(k,2), 'ZData', b(k,3));
    
    if(t(k) <= te)
        set(uavTrace,'XData', b(1:k,1),'YData', b(1:k,2), 'ZData', b(1:k,3));
        set(epsilonNeigborhood,'XData', b(k,1) + circle(:,1),...
            'YData', b(k,2) + circle(:,2), 'ZData', b(k,3) + circle(:,3));
    else
        set(epsilonNeigborhood,'visible','off');
    end

    set(quad,'XData', z(k,1),'YData', z(k,2), 'ZData', z(k,3));
    set(quadTrace,'XData', z(1:k,1),'YData', z(1:k,2), 'ZData', z(1:k,3));
    
    pause(0.1);
end

%% Phase II: Return
% 
% if(isempty(te)) % if the robot failed to catch the bug
%     % Decoupling the augmented state vector z from only phase I
%     t = t_I;
%     r = z_I(:,1:4);
%     b = z_I(:,[5, 6]);
% else
%     tspan_II = [te, tspan_I(end)];
%     r0_II = z_I(end,1:4)';
% 
%     [tk, c] = dist(tspan_II, [0.1 0.5], 2);
%     g = @(t)[0; 0; c(find(tk <= t, 1, 'last'),:)'];
% 
%     rd = zeros(4,1); ud = zeros(2,1);
% 
%     [t_II, r_II] = ode45(@(t,r) dr(t, r, u(r, rd, ud, K), g(t) ),...
%         tspan_II, r0_II);
% 
% 
%     % Decoupling the augmented state vector z from phase I and phase II
%     t = [t_I; t_II];
%     r = [z_I(:,1:4); r_II];
%     b = [z_I(:,[5, 6]); r_II(:,[1,2])]; % Since the bug is captured by the robot,
%                                   % bug's states are equal to the robot's.
% end

%% External Functions

function dv = augmentedSystem(t, v, dz, db, u, K)

% Decouple r and b states from the augmented state z
z = v(1:12, 1);
b = v([13,14,15], 1);
z0 = [0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
ud = @(t) [3;3;3;3];
dv = [ dz(t, z, u(z, [db(t, b);0;0;0;0;0;0;0;0;0], ud(t) , K) ) ; db(t, b) ];

end


function [value,isterminal,direction] = catchBug(t, v, epsilon)

value = norm( v([13, 14 , 15]) - v([1, 2, 3]) ) > epsilon;
isterminal = 1; % = 1 -> the integration is to terminate.
direction = 0;

end


% function [tk, c] = dist(tspan, tau, gamma)
% 
% % tspan: The domain of the piecewise constant function 
% % tau = [tau_min, tau_max]: minimum and maximum time intervals
% % gamma: Maximum norm of the c at every t
% 
% dim = 2;    % The dimension of c at every t
% p = 2;      % p-norm of c at every t. p = 2 -> l2 norm or Euclidean norm
% 
% delta_t = diff(tau);
% 
% tk = tspan(1);
% while(tk(end) < tspan(2))
%     tk(end+1,1) = tk(end) + delta_t*rand + tau(1);
% end
% 
% N = length(tk);
% c = 2*rand(N-1, dim) - 1;
% c = gamma*rand(N-1, 1).*(c./vecnorm(c,p,2));
% end




% %% Plot the position and velocity
% % Extract position and velocity components from the state vector
% position = z(:, 1:3);
% velocity = z(:, 4:6);
% 
% % Plot the position over time
% figure;
% subplot(2, 1, 1);
% plot(t, position, 'LineWidth', 2);
% title('Quadrotor Position Over Time');
% xlabel('Time (s)');
% ylabel('Position');
% legend('X', 'Y', 'Z');
% grid on;
% 
% % Plot the velocity over time
% subplot(2, 1, 2);
% plot(t, velocity, 'LineWidth', 2);
% title('Quadrotor Velocity Over Time');
% xlabel('Time (s)');
% ylabel('Velocity');
% legend('Vx', 'Vy', 'Vz');
% grid on;
% 
% % Extract displacement components from the state vector
% displacement = z(:, 1:3);
% 
% % Plot the displacement over time
% figure;
% plot(t, displacement, 'LineWidth', 2);
% title('Quadrotor Displacement Over Time');
% xlabel('Time (s)');
% ylabel('Displacement');
% legend('X', 'Y', 'Z');
% grid on;
% %% Plotting error Norm curve
% % Initialize variables to store error and norm
% error_vector = zeros(size(z));
% error_norm = zeros(size(z, 1), 1);
% 
% % Compute the error and norm at each time step
% for i = 1:size(z, 1)
%     error_vector(i, :) = z(i, 1:12) - zd';
%     error_norm(i) = norm(error_vector(i, :), 2);
% end
% 
% % Plot the error norm curve
% figure;
% plot(t, error_norm, '-x');
% title('Error Norm Curve');
% xlabel('Time (s)');
% ylabel('L2 Norm of Error');
% grid on;
% 
% %% State Trajectory
% figure;
% plot(t, z(:, 1:6), 'LineWidth', 2);  % Actual state trajectories
% hold on;
% plot(t, repmat(zd(1:6)', size(z, 1), 1), '--', 'LineWidth', 2);  % Desired state trajectories
% title('State Trajectories');
% xlabel('Time (s)');
% ylabel('State Value');
% legend('Actual', 'Desired');
% grid on;
% 
% %% Eigen Value Plot
% 
% closed_loop_matrix = A - B * K;
% eigenvalues = eig(closed_loop_matrix);
% figure;
% plot(real(eigenvalues), imag(eigenvalues), 'x');
% title('Eigenvalue Plot');
% xlabel('Real Part');
% ylabel('Imaginary Part');
% grid on;
% 
% %% Control input plots
% %Plot the control inputs
% figure;
% hold on;
% for i = 1:size(K, 1)
%     plot(t, u(z.', zd,ud, K(i, :)), 'LineWidth', 2, 'DisplayName', sprintf('Control Input %d', i));
% end
% hold off;
% 
% title('Control Inputs Over Time');
% xlabel('Time (s)');
% ylabel('Control Input');
% legend('show');
% grid on;
% 
% % % Initialize matrix to store control inputs
% % control_inputs = zeros(length(t), size(K, 1));
% % 
% % % Compute and store control inputs
% % for i = 1:size(K, 4)
% %     control_inputs(:, :,:,i) = u(z.', zd, ud, K(i, :)).';
% % end
% % 
% % % Plot the individual control inputs
% % figure;
% % plot(t, control_inputs, 'LineWidth', 2);
% % title('Control Inputs Over Time');
% % xlabel('Time (s)');
% % ylabel('Control Input');
% % legend('Control Input 1', 'Control Input 2', 'Control Input 3', 'Control Input 4');
% grid on;
