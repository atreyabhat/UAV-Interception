function [ s ] = init_state( start, yaw )
%INIT_STATE Initialize 13 x 1 state vector

s     = zeros(12,1);
phi0   = 0.0;
theta0 = 0.0;
psi0   = yaw;
Rot0   = RPYtoRot_ZXY(phi0, theta0, psi0);
%Quat0  = RotToQuat(Rot0);
s(1)  = start(1); %x
s(2)  = start(2); %y
s(3)  = start(3); %z
s(4)  = 0;        %phi
s(5)  = 0;        %theta
s(6)  = 0;        %psi
s(7)  = 0;        %vx
s(8)  = 0;        %vy
s(9)  = 0;        %vz
s(10) = 0;        %w1
s(11) = 0;        %w2
s(12) = 0;        %w3

end