function K = controller()
m = 0.5; % kg
g = 9.81; % m/s/s
l = 0.2; %m

sigma = 0.01;
I11 = 1.24;
I22 = I11;
I33 = 2.48;

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

% % Define the weighting matrices
Q = diag([1,1, 5, 0.1,0.1, 0, 1,1,3,1,1, 0, 0.00001]);
R = eye(4) * 10 ;

% % Call lqi with matrices A, B, C, D
% [K, ~, ~] = lqr(A,B,Q,R);
    % Q = diag([0.01,2000,100,0,100,0,1,0,0,0,0,0,0]); % State penalty
    % R = (1*10^-3)*eye(4,4);  % Control penalty


C = ones(1,12);
D = [0 0 0 0];
[K, ~, ~] = lqi(ss(A,B,C,D),Q,R,0);



