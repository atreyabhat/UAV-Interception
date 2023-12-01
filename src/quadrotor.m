clc; clear; close all;

%% Define the Parameters
m = 0.5; % kg
g = 9.81; % m/s/s
l = 0.2; %m
I = [1.24, 1.24, 2.48];
I11 = 1.24;
I22 = I11;
I33 = 2.48;
minF = 0.0;
maxF = 3.0;
sigma = 0.01;
mu = 3.0;
r = [0; 0; 0];
n = [0; 0; 0];
% u = [1; 0.9; 1.9; 1.5];

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

p = [g l m I mu sigma];

%% Robot equations
dz = @(t, z, u) dynamics(t, z, u, p, r, n);

u = @(z, zd, ud, K) ud + K*(zd - z);

%% Bug kinematics:

% a = 0.3; w = 10;
% db = @(t,b) [a*w*cos(t*w)*cos(t) - sin(t)*(a*sin(t*w) + 1);
%              cos(t)*(a*sin(t*w) + 1) + a*w*cos(t*w)*sin(t);
%              1];
% scale = rand(3, 1)*5; % Generate random multipliers for x, y, and z axes
db = @(t, b) [0.5;sin(t);0.5];

%% Controller setup/cost

% Q = diag([30, 30, 30, 30, 30, 30,30,30,30,30,30,30]);  % State cost
Q = diag([1,1,1,1,1,1,1,1,1,1,1,1]);
R = eye(4);  % Control cost
[K, ~, ~] = lqr(A, B, Q, R);

% p = [-1, -1, -2, -2, -2, -3, -1, -0.5, -0.5, -0.2, -0.2, -0.4];
% K = place(A, B, p);

%% Initial Conditions
z0 = [0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
b0 = [1;1;2];

%% Final states
zf = [0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
uf = [1;1;1;1]*m*g/4;


%% Drone Body
drone_body = [ 0.265,      0,     0, 1; ...
                    0, -0.265,     0, 1; ...
               -0.265,      0,     0, 1; ...
                    0,  0.265,     0, 1]';

%% Problem parameters
epsilon = 0.1;


%% Phase I: Pursue
tspan_I = [0 20];
v0_I = [z0; b0];

options = odeset('Event', @(t,v) catchBug(t, v, epsilon),...
    'RelTol',1e-6);

[t_I, v_I, te, ve] = ode45( @(t, v) augmentedSystem(t, v, dz, db, u, K),...
    tspan_I, v0_I, options);

%% Phase II: Return

if(isempty(te)) % if the robot failed to catch the bug
    % Decoupling the augmented state vector z from only phase I
    t = t_I;
    z = v_I(:,1:12);
    b = v_I(:,[13, 14, 15]);
else
    tspan_II = [te, tspan_I(end)];
    z0_II = v_I(end,1:12)';
    
    % [tk, c] = dist(tspan_II, [0.1 0.5], 2);
    % g = @(t)[0; 0; c(find(tk <= t, 1, 'last'),:)'];
    
    % zd = zeros(4,1); ud = zeros(2,1);
    
    [t_II, z_II] = ode45(@(t,z) dz(t, z, u(z, zf, uf, K) ),...
        tspan_II, z0_II);
    % disp(zf')
    disp(z_II)
    
    
    % Decoupling the augmented state vector z from phase I and phase II
    t = [t_I; t_II];
    z = [v_I(:,1:12); z_II];
    b = [v_I(:,[13, 14, 15]); z_II(:,[1, 2, 3])]; % Since the bug is captured by the robot,
                                  % bug's states are equal to the robot's.
end

%% Init. 3D Fig.
fig1 = figure('pos',[0 200 800 800]);
h = gca;
view(3);
axis equal;
grid on;
xlim([-2 15]);
ylim([-2 15]);
zlim([-10 20]);
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
% disp(z)

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
		'XData', z(k,1), ...
		'YData', z(k,2),'ZData',0);

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


%% External Functions

function dv = augmentedSystem(t, v, dz, db, u, K)

% Decouple r and b states from the augmented state z
z = v(1:12, 1);
b = v([13,14,15], 1);
m = 0.5; % kg
g = 9.81;
ud = @(t) [1;1;1;1]*m*g/4;

dv = [ dz(t, z, u(z, [b;0;0;0;db(t,b);0;0;0], ud(t) , K) ) ; db(t, b) ];

end


function [value,isterminal,direction] = catchBug(t, v, epsilon)

value = norm( v([13, 14 , 15]) - v([1, 2, 3]) ) > epsilon;
isterminal = 1; % = 1 -> the integration is to terminate.
direction = 0;

end


