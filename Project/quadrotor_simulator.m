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
epsilon = 0.2;


%% Control Function
dz = @(t, z, u) dynamics(t, z, u, p, r, n);

u = @(z, zd, ud, K) ud + K*(zd - z);

K = controller();

%% UAV Path

bt = linspace(0, 10, 500);  % Adjust the time span and number of points as needed

% Non-linear function to define robot motion in 3D space
position_function = @(t) [t*0.8; t*0.8; t*1];

% Evaluate the function at each time point
% uav_position = position_function(bt)';

% disp(uav_position)
%% Initial Conditions
z0 = [5; 4; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
b0 = [1;1;2];

%% Final states
zf = [5; 3; 5; 0; 0; 0; 0; 0; 0; 0; 0; 0];
uf = zeros(4, 1);

%% ODE Solver

t_span = [0 10];

% [t,z] = ode45(@(t,z) quadrotor(t, z, u(z, [position_function(t)+[0.05;0.05;0];0;0;0;0;0;0;0;0;0], ud, K), p, r, n), t_span, z0);

% disp(z)

%% Phase I: Pursue
tspan_I = [0 10];

options = odeset('Event', @(t,z) catchuav(t, z, position_function(t), epsilon),...
    'RelTol',1e-6);

[t_I, z_I, te, ve] = ode45( @(t, z) quadrotor(t, z, ...
    u(z, [position_function(t)+[0.5;0.5;0];0;0;0;0;0;0;0;0;0], ud, K), ...
    p, r, n), t_span, z0, options);
disp(te);
t = t_I;
z = z_I;

for i = 1:length(t)
    % Evaluate the function at the current time point
    uav_position(i, :) = position_function(t(i));
end
% disp(uav_position);

% %% Phase II: Return
% 
% if(isempty(te)) % if the robot failed to catch the bug
%     % Decoupling the augmented state vector z from only phase I
%     t = t_I;
%     z = v_I(:,1:12);
%     b = v_I(:,[13, 14, 15]);
% else
%     tspan_II = [te, tspan_I(end)];
%     z0_II = v_I(end,1:12)';
% 
%     % [tk, c] = dist(tspan_II, [0.1 0.5], 2);
%     % g = @(t)[0; 0; c(find(tk <= t, 1, 'last'),:)'];
% 
%     % zd = zeros(4,1); ud = zeros(2,1);
% 
%     [t_II, z_II] = ode45(@(t,z) dz(t, z, u(z, zf, uf, K) ),...
%         tspan_II, z0_II);
%     % disp(zf')
%     disp(z_II)
% 
% 
%     % Decoupling the augmented state vector z from phase I and phase II
%     t = [t_I; t_II];
%     z = [v_I(:,1:12); z_II];
%     b = [v_I(:,[13, 14, 15]); z_II(:,[1, 2, 3])]; % Since the bug is captured by the robot,
%                                   % bug's states are equal to the robot's.
% end


%% Plotting the results

% for i=1:4
%     ax(i) = subplot(2,2,i,'NextPlot','Add','Box','on','XGrid','on','YGrid','on',...
%                 'Xlim',[t(1), t(end)],...
%                 'TickLabelInterpreter','LaTeX','FontSize',14);
%     xlabel('t','Interpreter','LaTeX','FontSize',14);        
% end

% 
% plot(ax(1), t,z(:,1:3), 'LineWidth', 1.5);
% legend(ax(1), {'$x_1$', '$x_2$', '$x_3$'},... 
%     'Interpreter', 'LaTeX', 'FontSize', 14);
% title(ax(1), '${\bf x}$','Interpreter','LaTeX','FontSize',14);
% xlabel(ax(1), 't','Interpreter','LaTeX','FontSize',14);
% 
% plot(ax(3), t, z(:,4:6), 'LineWidth', 1.5);
% legend(ax(3), {'$\phi$', '$\theta$', '$\psi$'},...
%     'Interpreter', 'LaTeX', 'FontSize', 14);
% title(ax(3), '\boldmath$\alpha$','Interpreter','LaTeX','FontSize',14);
% 
% plot(ax(2), t, z(:,7:9), 'LineWidth', 1.5);
% legend(ax(2), {'$\dot{x}_1$', '$\dot{x}_2$', '$\dot{x}_3$'},...
%     'Interpreter', 'LaTeX', 'FontSize', 14);
% title(ax(2), '$\dot{\bf x}$','Interpreter','LaTeX','FontSize',14);
% 
% plot(ax(4), t, z(:,10:12), 'LineWidth', 1.5);
% legend(ax(4), {'$\omega_1$', '$\omega_2$', '$\omega_3$'},...
%     'Interpreter', 'LaTeX', 'FontSize', 14);
% title(ax(4), '\boldmath$\omega$','Interpreter','LaTeX','FontSize',14);



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
uavTrace = plot3(animation_axes,0,0,0, '-', 'Color', uavColor);
quadTrace = plot3(animation_axes, 0, 0, 1, '-', 'Color', quadrotorColor);
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
    set(uavTrace,'XData', uav_position(1:k,1),'YData', uav_position(1:k,2), 'ZData', uav_position(1:k,3));
    pause(t(k)-toc);
    pause(0.01);
end


function [value,isterminal,direction] = catchuav(t, z, b, epsilon)

value = norm( z([1,2,3]) - b([1, 2, 3]) ) > epsilon;
isterminal = 1; % = 1 -> the integration is to terminate.
direction = 0;

end