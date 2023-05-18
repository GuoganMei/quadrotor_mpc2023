function FormulateNolinearStatefunction()
%% import CasADi
addpath('D:\matlab_mpt_tbx_canrm\casadi-3.6.1-windows64-matlab2018b');
import casadi.*
%% define symbolic variables for modeling
x = SX.sym('x');y = SX.sym('y');z = SX.sym('z');
phi = SX.sym('phi');theta = SX.sym('theta');psi = SX.sym('psi');
xdot = SX.sym('xdot');ydot = SX.sym('ydot');zdot = SX.sym('zdot');
phidot = SX.sym('phidot');thetadot = SX.sym('thetadot');psidot = SX.sym('psidot');
state = [x,y,z,phi,theta,psi,xdot,ydot,zdot,phidot,thetadot,psidot]';
u1 = SX.sym('u1');u2 = SX.sym('u2');u3 = SX.sym('u3');u4 = SX.sym('u4');
control = [u1,u2,u3,u4]';
%% define dynamics parameters
Ixx = 1.2;Iyy = 1.2;Izz = 2.3;
k = 1;l = 0.25;m = 2;b = 0.2; g = 9.81;
%% define system dynamics
% transformation matrix for angular velocities from inertial frame to rates of euler angles
Rz = [cos(psi), -sin(psi), 0;
    sin(psi), cos(psi), 0;
    0, 0, 1];
Ry = [cos(theta), 0, sin(theta);
    0, 1, 0;
    -sin(theta), 0, cos(theta)];
Rx = [1, 0, 0;
    0, cos(phi), -sin(phi);
    0, sin(phi), cos(phi)];

R = Rx*Ry*Rz;
W = [cos(theta)*cos(psi),sin(psi),0;
-cos(theta)*sin(psi),cos(psi),0;
sin(theta),0,1];

% Jacobian (relates body frame angular velocities to the rates of euler angles)
I = [Ixx, 0, 0; 0, Iyy, 0; 0, 0, Izz];
J = W.'*I*W;

% Coriolis forces
dJ_deulerangle_flat = jacobian(J,[phi,theta,psi]');
dJ_dt_flat = dJ_deulerangle_flat*[phidot,thetadot,psidot]';
dJ_dt = reshape(dJ_dt_flat,3,3);

etadot_J = [phidot,thetadot,psidot]*J;
grad_etadot_J = jacobian(etadot_J,[phi,theta,psi]');
C = dJ_dt - 1/2*grad_etadot_J.';
% Torques in the direction of phi, theta, psi
tau_beta = [l*k*(-u2 + u4);l*k*(-u1 + u3);b*(-u1+u2-u3+u4)];
% tau_beta = [l*k*(u2 - u4);l*k*(-u1 + u3);b*(-u1+u2-u3+u4)];
% Total thrust
Thrust = k*(u1+u2+u3+u4);

% Dynamics
rhs = SX.sym('rhs',12,1);
rhs(1) = xdot;
rhs(2) = ydot;
rhs(3) = zdot;
rhs(4) = phidot;
rhs(5) = thetadot;
rhs(6) = psidot;

% Equations for COM configuration
rhs(7:9) = -g*[0;0;1] + R*[0;0;Thrust]/m;

% Euler Lagrange equations for angular dynamics
rhs(10:12) = inv(J)*(tau_beta - C*[phidot; thetadot; psidot]);



% build function object
f = Function('f',{state,control},{rhs});

% dynamics derivative
% A = jacobian(rhs,state);
% B = jacobian(rhs,control);
% dfdx_dfdu = Function('dfdx_dfdu',{state,control},{A,B});
% [A_,B_]=dfdx_dfdu(zeros(12,1),g/2*ones(4,1));
% A_=full(A_);
% B_=full(B_);

save NolinearStatefunction.mat f; 
clear;
end