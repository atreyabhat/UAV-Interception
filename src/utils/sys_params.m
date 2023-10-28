function params = sys_params()
% SYS_PARAMS basic parameters for the quadrotor

m = 0.5; % kg
g = 9.81; % m/s/s
l = 0.2; %m
I = [1.24, 0,     0;
     0,   1.24,   0;
     0,   0,   2.48];

params.mass = m;
params.I    = I;
params.invI = inv(I);
params.gravity = g;
params.arm_length = l;

params.minF = 0.0;
params.maxF = 3.0;
params.sigma = 0.01;

end
