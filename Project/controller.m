function K = controller()
m = 0.5; % kg
g = 9.81; % m/s/s
l = 0.2; %m
I11 = 1.24;
I22 = I11;
I33 = 2.48;
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

Q = diag([1, 1, 1,0.1,0.1,0.1,0,0,0,0,0,0])*14;
R = eye(4) * 5;  % Control cost
[K, ~, ~] = lqr(A, B, Q, R);