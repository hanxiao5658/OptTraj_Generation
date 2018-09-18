function dz = LG_cartPoleDynamics(z,u)
%% 简化后得到的辨识模型
% dz = cartPoleDynamics(z,u,p)
%
% This function computes the first-order dynamics of the cart-pole.
%
% INPUTS:
%   z = [4, n] = [x;q;dx;dq] = state of the system
%   u = [1, n] = horizontal force applied to the cart
%   p = parameter struct
%       .g = gravity
%       .m1 = cart mass
%       .m2 = pole mass
%       .l = pendulum length
% OUTPUTS:
%   dz = dz/dt = time derivative of state
%
%
% X = [yQ,dyQ,r,dr];u = ayQ;
L = 0.8;
g = 9.81;

% yQ = z(1,:);
% dyQ = z(2,:);
% ye = z(3,:);
% dye = z(4,:);
% 
% yQdot = dyQ;
% dyQdot = u;
% yedot = dye;
K = -1.3477;
wn = 4.1268;
zeta = 0.0634;

A = [0,1,0,0;...
    0,0,0,0;...
    0,0,0,1;...
    0,0,-wn^2,-2*wn*zeta];
B = [0;1;0;K];

dz = A*z + B*u;

% M = (1/2).*(2+2.*ye.^2.*(L.^2+(-1).*ye.^2).^(-1));
% fq = u+ye.*(dye.^2.*L.^2.*(L.^2+(-1).*ye.^2).^(-2)+g.*(L.^2+(-1).* ...
%   ye.^2).^(-1/2));
% dyedot = -M.\fq;
% dz = [yQdot;dyQdot;yedot;dyedot];
end