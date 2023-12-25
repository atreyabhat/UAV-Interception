%% Initializations
clc; clear; close all;

g = 9.81;   % The gravitational acceleration [m/s^2]
l =  0.2;   % Distance from the center of mass to each rotor [m]
m =  0.5;   % Total mass of the quadrotor [kg]
I = [1.24, 1.24, 2.48]; % Mass moment of inertia [kg m^2]
mu = 3.0;   % Maximum thrust of each rotor [N]
sigma = 0.01; % The proportionality constant relating thrust to torque [m]


p = [g l m I mu sigma];

r = [0; 0; 0];
n = [0; 0; 0];
ud = [1;1;1;1] * m*g/4;
%% Problem parameters
epsilon = 0.1;

% ki = K*0.01;

%% Control Function
dz = @(t, z, u) quadrotor(t, z, u, p, r, n);

u = @(z, zd, ud, K) ud + K(:,1:12)*(zd - z) + K(:,13) * [z(3)];

K = controller();
% disp(K(:,13));
%% UAV Path

bt = linspace(0, 10, 500);  % Adjust the time span and number of points as needed

% Non-linear function to define robot motion in 3D space
% position_function = @(t) [t*1.8; t*0.8; t*0];
position_function = @(t) [t; t ; 7];
dt = 1;
position_function_est = @(t) [t+3;t+3;7];

%% Initial Conditions
z0 = [0; 0; 0.1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
b0 = [1;1;2];

%% Final states
zf = [0; 0; 0.1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
uf = zeros(4, 1);

%% ODE Solver

t_span = [0 20];

%% Phase I: Pursue

options = odeset('Event', @(t,z) catchuav(t, z, position_function(t), epsilon),...
    'RelTol',1e-6);


% zd = @(t) [0.1*t; 1; 1 ;zeros(9,1)];

% [t, z] = ode45( @(t, z) quadrotor(t, z, ...
%     u(z, zd(t) , ud, K), ...
%     p, r, n), t_span, z0);

[t_I, z_I, te, ze] = ode45( @(t, z) quadrotor(t, z, ...
    u(z, [position_function_est(t);0;0;0;0;0;0;0;0;0], ud, K), ...
    p, r, n, t+2, 0), t_span, z0,  options);
if(isempty(te))
    te = 0;
end
uavfinalpos = position_function(te);
if uavfinalpos(1) >= 0 && uavfinalpos(1) <= 10 && uavfinalpos(2) >= 0 && ... 
    uavfinalpos(2) <= 10 && uavfinalpos(3) >= 0 && uavfinalpos(3) <= 10 && te ~= 0
    te = te;
else
    te = 0;
end

for i = 1:length(t_I)
    % Evaluate the function at the current time point
    uav_position_I(i, :) = position_function(t_I(i));
end
% disp(uav_position);

%% Phase II: Return

if te == 0 % if the robot failed to catch the bug
    % Decoupling the augmented state vector z from only phase I
    tspan_II = [t_I(end) 25];
    z0_II = z_I(end,:)';
    [t_II, z_II] = ode45(@(t,z) quadrotor(t, z, ...
    u(z, zf, ud, K), p, r, n,0,0), tspan_II, z0_II);
    t = [t_I; t_II];
    z = [z_I; z_II];
    for i = 1:length(t_II)
    % Evaluate the function at the current time point
        uav_position_II(i, :) = position_function(t_II(i));
    end
    uav_position = [uav_position_I; uav_position_II];
else
    tspan_II = [te, t_span(end)];
    z0_II = z_I(end,:)';

    [tk, c] = dist(tspan_II, [0.1 0.5], 2);
    g = @(t)[0; 0; c(find(tk <= t, 1, 'last'),:)'];

    % zd = zeros(4,1); ud = zeros(2,1);

    [t_II, z_II] = ode45(@(t,z) quadrotor(t, z, ...
    u(z, zf, ud, K), p, r, n,0,0), tspan_II, z0_II);
    % disp(zf')
    % disp(z_II)


    % Decoupling the augmented state vector z from phase I and phase II
    t = [t_I; t_II];
    z = [z_I; z_II];
    uav_position = [uav_position_I; z_II(:,[1,2,3])];
end


%% Plotting the results
% plotting(z,t);
for i=1:4
    ax(i) = subplot(2,2,i,'NextPlot','Add','Box','on','XGrid','on','YGrid','on',...
                'Xlim',[t(1), t(end)],...
                'TickLabelInterpreter','LaTeX','FontSize',14);
    xlabel('t','Interpreter','LaTeX','FontSize',14);        
end


plot(ax(1), t,z(:,1:3), 'LineWidth', 1.5);
legend(ax(1), {'$x_1$', '$x_2$', '$x_3$'},... 
    'Interpreter', 'LaTeX', 'FontSize', 14);
title(ax(1), '${\bf x}$','Interpreter','LaTeX','FontSize',14);
xlabel(ax(1), 't','Interpreter','LaTeX','FontSize',14);

plot(ax(3), t, z(:,4:6), 'LineWidth', 1.5);
legend(ax(3), {'$\phi$', '$\theta$', '$\psi$'},...
    'Interpreter', 'LaTeX', 'FontSize', 14);
title(ax(3), '\boldmath$\alpha$','Interpreter','LaTeX','FontSize',14);

plot(ax(2), t, z(:,7:9), 'LineWidth', 1.5);
legend(ax(2), {'$\dot{x}_1$', '$\dot{x}_2$', '$\dot{x}_3$'},...
    'Interpreter', 'LaTeX', 'FontSize', 14);
title(ax(2), '$\dot{\bf x}$','Interpreter','LaTeX','FontSize',14);

plot(ax(4), t, z(:,10:12), 'LineWidth', 1.5);
legend(ax(4), {'$\omega_1$', '$\omega_2$', '$\omega_3$'},...
    'Interpreter', 'LaTeX', 'FontSize', 14);
title(ax(4), '\boldmath$\omega$','Interpreter','LaTeX','FontSize',14);



%% Animation
animation_fig = figure;

airspace_box_length = 10;

animation_axes = axes('Parent', animation_fig,...
    'NextPlot','add','DataAspectRatio',[1 1 1],...
    'Xlim',airspace_box_length*[-0.1 1],...
    'Ylim',airspace_box_length*[-0.1 1],...
    'Zlim',airspace_box_length*[0 1],...
    'box','on','Xgrid','on','Ygrid','on','Zgrid','on',...
    'TickLabelInterpreter','LaTeX','FontSize',14);

xlabel('X');
ylabel('Y');                                                                                                                                                                                                                                 
zlabel('Z');
view(animation_axes, 3);

N = 10;
Q = linspace(0,2*pi,N)';
circle = 0.3*l*[cos(Q) sin(Q) zeros(N,1)];
loc = l*[1 0 0; 0 1 0; -1 0 0; 0 -1 0];
quadrotorColor = [0, 0.4 0.7];
uavColor = [0.9, 0.3 0.1];

silhouette = plot3(0,0,0, '--', 'Color', 0.5*[1 1 1], 'LineWidth', 1 ,...
    'Parent', animation_axes);
body = plot3(0,0,0, 'Color',lines(1), 'LineWidth', 2,...
        'Parent', animation_axes);
uav = plot3(animation_axes,0,0,0, 'o', 'Color', uavColor,...
    'MarkerFaceColor', uavColor, 'MarkerSize', 5,...
    'DisplayName','Bug');
uavTrace = plot3(animation_axes,0,0,0, '-', 'Color', uavColor, 'LineWidth',3);
quadTrace = plot3(animation_axes, 0, 0, 1, '-', 'Color', 'g', 'LineWidth',3);
epsilonNeigborhood = plot3(animation_axes, 0, 0, 0, 'k', 'LineWidth', 1);
if te == 0
    te = inf;
end
Q = linspace(0, 2*pi, 15)';
circle_range = epsilon*[cos(Q) sin(Q) cos(Q)];
for i=1:4
    rotor(i) = plot3(0,0,0, 'Color', lines(1), 'LineWidth', 2,...
        'Parent', animation_axes);
end
disp(te)
tic;
for k=1:length(t)
    
    R = [ cos(z(k,5))*cos(z(k,6)), sin(z(k,4))*sin(z(k,5))*cos(z(k,6)) - cos(z(k,4))*sin(z(k,6)), sin(z(k,4))*sin(z(k,6)) + cos(z(k,4))*sin(z(k,5))*cos(z(k,6));
          cos(z(k,5))*sin(z(k,6)), cos(z(k,4))*cos(z(k,6)) + sin(z(k,4))*sin(z(k,5))*sin(z(k,6)), cos(z(k,4))*sin(z(k,5))*sin(z(k,6)) - sin(z(k,4))*cos(z(k,6));
                     -sin(z(k,5)),                                 sin(z(k,4))*cos(z(k,5)),                                 cos(z(k,4))*cos(z(k,5))];
    for i=1:4
        ctr(i,:) = z(k,1:3) + loc(i,:)*R';
        pose = ones(N,1)*z(k,1:3) + (ones(N,1)*loc(i,:) + circle)*R';
        set(rotor(i), 'XData', pose(:,1), 'YData', pose(:,2),  'ZData', pose(:,3) );
         
    end
    set(silhouette,'XData', [0, z(k,1), z(k,1), z(k,1)],...
        'YData', [0, 0, z(k,2), z(k,2)],...
        'ZData', [0, 0, 0, z(k,3)]);
    set(body, 'XData', [ctr([1 3],1); NaN; ctr([2 4],1)], ...
        'YData', [ctr([1 3],2); NaN; ctr([2 4],2)],...
        'ZData', [ctr([1 3],3); NaN; ctr([2 4],3)] );
    set(quadTrace,'XData', z(1:k,1),'YData', z(1:k,2), 'ZData', z(1:k,3));
    set(uav,'XData', uav_position(k,1),'YData', uav_position(k,2), 'ZData', uav_position(k,3));

    if(t(k) <= te)
        set(uavTrace,'XData', uav_position(1:k,1),'YData', uav_position(1:k,2), 'ZData', uav_position(1:k,3));
        set(epsilonNeigborhood,'XData', uav_position(k,1) + circle_range(:,1),...
            'YData', uav_position(k,2) + circle_range(:,2), 'ZData', uav_position(k,3) + circle_range(:,3));
    else
        set(epsilonNeigborhood,'visible','off');
    end
    % set(uavTrace,'XData', uav_position(1:k,1),'YData', uav_position(1:k,2), 'ZData', uav_position(1:k,3));
    pause(t(k)-toc);
    pause(0.01);
end


function [value,isterminal,direction] = catchuav(t, z, uav, epsilon)

v1 = norm( z([1,2,3]) - uav([1, 2, 3]) ) > epsilon;
if uav(1) >= 0 && uav(1) <= 10 && uav(2) >= 0 && uav(2) <= 10 && uav(3) >= 0 && uav(3) <= 10
    v2 = 1;
else
    v2 = 0;
end
if v1 == 1 && v2 == 1
    value = 1;
else
    value = 0;
end
isterminal = 1; % = 1 -> the integration is to terminate.
direction = 0;

end

function [tk, c] = dist(tspan, tau, gamma)

% tspan: The domain of the piecewise constant function 
% tau = [tau_min, tau_max]: minimum and maximum time intervals
% gamma: Maximum norm of the c at every t

dim = 3;    % The dimension of c at every t
p = 3;      % p-norm of c at every t. p = 2 -> l2 norm or Euclidean norm

delta_t = diff(tau);

tk = tspan(1);
while(tk(end) < tspan(2))
    tk(end+1,1) = tk(end) + delta_t*rand + tau(1);
end

N = length(tk);
c = 2*rand(N-1, dim) - 1;
c = gamma*rand(N-1, 1).*(c./vecnorm(c,p,2));
end