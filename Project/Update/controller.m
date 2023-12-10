function K = controller()
m = 0.5; % kg
g = 9.81; % m/s/s
l = 0.2; %m
Ixx = 1.24;
Iyy = Ixx;
Izz = 2.48;
sigma = 0.01;


% % Define A matrix
% A11 = zeros(6, 6);
% A12 = eye(6);
% A13 = zeros(6, 3);
% A22 = zeros(6,9);
% A21 = [0, 0, 0, 0, g, 0;
%        0, 0, 0, -g, 0, 0;
%        0, 0, 0, 0, 0, 0;
%        0, 0, 0, 0, 0, 0;
%        0, 0, 0, 0, 0, 0;
%        0, 0, 0, 0, 0, 0;
%        -1, 0, 0, 0, 0, 0;
%        0, -1, 0, 0, 0, 0;
%        0, 0, -1, 0, 0, 0];
% 
% A = [A11, A12, A13; A21, A22];
% disp(A)
% 
% % Define B matrix
% B1 = zeros(6, 4);
% B2 = [0, 0, 0, 0;
%       0, 0, 0, 0; 
%       1/m, 1/m,1/m, 1/m; 
%       0, l/I11,0, -l/I11; 
%       -l/I22,0, l/I22,0; 
%       sigma/I33, -sigma/I33, sigma/I33, -sigma/I33];
% 
% 
% B = [B1; B2];

A = zeros(15, 15);

for i = 1:6
    A(i, i+6) = 1;
end

A(7, 5) = g;
A(8, 4) = -g;

A(13, 1) = 1;
A(14, 2) = 1;
A(15, 3) = 1;


B = zeros(15, 4);

B(9 , 1) = 1/m;
B(9 , 2) = 1/m;
B(9 , 3) = 1/m;
B(9 , 4) = 1/m;
B(10, 2) = 1/Ixx;
B(10, 4) = -1/Ixx;
B(11, 3) = l/Iyy;
B(11, 1) = -l/Iyy;
B(12, 1) = sigma/Izz;
B(12, 2) = -sigma/Izz;
B(12, 3) = sigma/Izz;
B(12, 4) = -sigma/Izz;


Q = diag([1, 1, 1,2,2,0.2,1,1,1,2,2,0.2,0.1,0.1,0.1]);
R = eye(4) * 5;  % Control cost
[K, ~, ~] = lqr(A, B, Q, R);