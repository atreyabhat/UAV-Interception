% simulation_3d.m
function [t_out, s_out] = simulation_3d(trajhandle, controlhandle)
    % NOTE: This script will not run as expected unless you fill in proper
    % code in trajhandle and controlhandle
    % You should not modify any part of this script except for the
    % visualization part

    addpath('utils');

    % real-time
    real_time = true;

    % max time
    max_time = 10;

    % parameters for simulation
    params = sys_params;

    %% **************************** FIGURES *****************************
    disp('Initializing figures...');
    h_fig = figure;
    h_3d = gca;
    axis equal
    grid on
    view(3);
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
    quadcolors = lines(1);

    set(gcf, 'Renderer', 'OpenGL')

    % Set fixed axis limits to prevent zooming
    %axis([-8 8 -8 8 0 8]);

    %% *********************** INITIAL CONDITIONS ***********************
    disp('Setting initial conditions...');
    tstep = 0.01; % this determines the time step at which the solution is given
    cstep = 0.05; % image capture time interval
    max_iter = max_time / cstep; % max iteration
    nstep = cstep / tstep;
    time = 0; % current time
    err = []; % runtime errors

    % Get start and stop position
    des_start = trajhandle(0, []);
    des_stop = trajhandle(inf, []);
    stop_pos = des_stop.pos;
    x0 = init_state(des_start.pos, 0);
    xtraj = zeros(max_iter * nstep, length(x0));
    ttraj = zeros(max_iter * nstep, 1);

    x = x0; % state

    pos_tol = 0.01;
    vel_tol = 0.01;

    % Initialize the point at the desired initial position
    point = [des_start.pos(1), des_start.pos(2), 1]; % Start 1 meter above the ground

    % Define a circular trajectory in the x-y plane
    radius = 3; % Adjust the radius as needed
    angular_velocity = 5* 2 * pi / max_time;
    
    QP = QuadPlot(1, x0, 0.1, 0.04, quadcolors(1,:), max_iter, h_3d); % Initialize QuadPlot with one quadrotor and a red point

    %% ************************* RUN SIMULATION *************************
    disp('Simulation Running....');
    % Main loop
    for iter = 1:max_iter

        timeint = time:tstep:time + cstep;

        tic;

        % Calculate the desired state using the trajhandle function
        current_state = stateToQd(x);
        desired_state = trajhandle(time + cstep, current_state);

        % Update the point's position based on the circular trajectory
        t = time / max_time; % Normalize time to [0, 1]
        angle = angular_velocity * time;
        point = [radius * cos(angle), radius * sin(angle), 2]; % Stay 1 meter above the ground

        % Update the QuadPlot with the new point position
        QP.UpdateQuadPlot(x, [desired_state.pos; desired_state.vel], time, point);

        % Run simulation
        [tsave, xsave] = ode45(@(t, s) quadEOM(t, s, controlhandle, trajhandle, params), timeint, x);
        x = xsave(end, :)';

        % Save to traj
        xtraj((iter - 1) * nstep + 1:iter * nstep, :) = xsave(1:end - 1, :);
        ttraj((iter - 1) * nstep + 1:iter * nstep) = tsave(1:end - 1);

        time = time + cstep; % Update simulation time
        t = toc;
        % Check to make sure ode45 is not timing out
        if (t > cstep * 500)
            err = 'Ode45 Unstable';
            break;
        end

        % Pause to make real-time
        if real_time && (t < cstep)
            pause(cstep - t);
        end

        % Check termination criteria
        if terminate_check(x, time, stop_pos, pos_tol, vel_tol, max_time)
            break;
        end
    end

    %% ************************* POST PROCESSING *************************
    % Truncate xtraj and ttraj
    xtraj = xtraj(1:iter * nstep, :);
    ttraj = ttraj(1:iter * nstep);

    if (~isempty(err))
        error(err);
    end

    disp('finished.');

    t_out = ttraj;
    s_out = xtraj;
end
