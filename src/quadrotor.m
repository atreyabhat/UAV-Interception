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

%% Controller setup/cost

Q = diag([2, 2, 2, 1, 1, 1,1,1,1,1,1,1]);  % State cost
R = eye(4);  % Control cost
[K, ~, ~] = lqr(A, B, Q, R);

%% Initial Conditions
z0 = [4; 4; 5; 0; 0; 0; 0; 0; 0; 0; 0; 0];
% u = [1,1,1,1] *m*g/4;

%% Desired states
zd = [0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
ud = zeros(4, 1);


%% Drone Body
drone_body = [ 0.265,      0,     0, 1; ...
                    0, -0.265,     0, 1; ...
               -0.265,      0,     0, 1; ...
                    0,  0.265,     0, 1]';
                    % 0,      0,     0, 1; ...
                    % 0,      0, -0.15, 1]';

%% Timespan for simulation

tspan = [0 10];
%% Solve system from every initial state
[t, z] = ode45( @(t, z) dz(t, z, u(z, zd, ud, K)), tspan, z0);

%% Init. 3D Fig.
fig1 = figure('pos',[0 200 800 800]);
h = gca;
view(3);
axis equal;
grid on;
xlim([-5 5]);
ylim([-5 5]);
zlim([0 8]);
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
% uav = plot3(0, 0, 0, 'o', 'Color', uavColor,...
%     'MarkerFaceColor', uavColor, 'MarkerSize', 10,...
%     'DisplayName','Bug');
% uavTrace = plot3(0, 0, 0, '-', 'Color', uavColor);

quad = plot3(gca, 4, 4, 5, 's', 'Color', quadrotorColor,...
    'MarkerFaceColor', quadrotorColor, 'MarkerSize', 1,...
    'DisplayName','Robot');
quadTrace = plot3(gca, 4, 4, 5, '-', 'Color', quadrotorColor);

% hold(gca, 'off');


%% Animate

for k=1:length(t)
    disp(z(k, 1:3))
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
		'XData', z(k,1), ...
		'YData', z(k,2),'ZData',0);

    set(quad,'XData', z(k,1),'YData', z(k,2), 'ZData', z(k,3));
    set(quadTrace,'XData', z(1:k,1),'YData', z(1:k,2), 'ZData', z(1:k,3));
    
    pause(0.1);
end










