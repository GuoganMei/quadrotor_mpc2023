clc;
clear;
close all;

FormulateNolinearStatefunction();
load('system.mat');
load('NolinearStatefunction');
g=9.81;
Ts=0.05;
p=25;

A12=zeros(12,1);
A12(9,1)=1;
A21=zeros(1,12);
A22=0;
B12=zeros(1,4);
A=[A,A12;A21,A22];
B=[B;B12];
[Ad,Bd]=discretize_state_function(A,B,Ts);

xmax=[2,2,2,1,1,1,10,10,10,4,4,4];
xmin=-xmax;
umax=[12,12,12,12];
umin=[0,0,0,0];
%x0=[0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,-g]';%regulation initial state 
x0=[1,-1,1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,-g]';%tracking initial state
x_withg=x0;
x=x_withg(1:12,1);

BL=B(9:12,:);
T=[g*BL(3,:);-g*BL(2,:);BL(1,:);BL(4,:)];
invT=inv(T);
G=[0;0;-g;0];

A1=[zeros(3,1),eye(3);zeros(1,4)];
B1=[zeros(3,1);1];
A2=[zeros(3,1),eye(3);zeros(1,4)];
B2=[zeros(3,1);1];
A3=[0,1;0,0];
B3=[0;1];
A4=[0,1;0,0];
B4=[0;1];

[A1d,B1d]=discretize_state_function(A1,B1,Ts);
[A2d,B2d]=discretize_state_function(A2,B2,Ts);
[A3d,B3d]=discretize_state_function(A3,B3,Ts);
[A4d,B4d]=discretize_state_function(A4,B4,Ts);


model_1=LTISystem('A',A1d,'B',B1d);
model_1.x.min=[xmin(1),xmin(7),g*xmin(5),g*xmin(11)];
model_1.x.max=[xmax(1),xmax(7),g*xmax(5),g*xmax(11)];
% model_1.x.with('reference');
% model_1.x.reference = 'free';
Q1=diag([700,10,70,0]);
R1=diag(0.1);
model_1.x.penalty = QuadFunction(Q1);
model_1.u.penalty= QuadFunction(R1);
mpc_1= MPCController(model_1, p);
%mpc_1 = mpc_1.toExplicit();

model_2=LTISystem('A',A2d,'B',B2d);
model_2.x.min=[xmin(2),xmin(8),g*xmin(4),g*xmin(10)];
model_2.x.max=[xmax(2),xmax(8),g*xmax(4),g*xmax(10)];
% model_2.x.with('reference');
% model_2.x.reference = 'free';
Q2=diag([700,10,70,0]);
R2=diag(0.1);
model_2.x.penalty = QuadFunction(Q2);
model_2.u.penalty= QuadFunction(R2);
mpc_2= MPCController(model_2, p);
%mpc_2 = mpc_2.toExplicit();

model_3=LTISystem('A',A3d,'B',B3d);
model_3.x.min=[xmin(3),xmin(9)];
model_3.x.max=[xmax(3),xmax(9)];
% model_3.x.with('reference');
% model_3.x.reference = 'free';
Q3=diag([700,10]);
R3=diag(0.1);
model_3.x.penalty = QuadFunction(Q3);
model_3.u.penalty= QuadFunction(R3);
mpc_3= MPCController(model_3, p);
mpc_3 = mpc_3.toExplicit();
figure;
mpc_3.partition.plot();

model_4=LTISystem('A',A4d,'B',B4d);
model_4.x.min=[xmin(6),xmin(12)];
model_4.x.max=[xmax(6),xmax(12)];
% model_4.x.with('reference');
% model_4.x.reference = 'free';
Q4=diag([70,0.5]);
R4=diag(0.1);
model_4.x.penalty = QuadFunction(Q4);
model_4.u.penalty= QuadFunction(R4);
mpc_4= MPCController(model_4, p);
mpc_4 = mpc_4.toExplicit();
figure;
mpc_4.partition.plot();

sim_time=10;
sim_steps=sim_time/Ts;

x_list=x;
u_list=zeros(4,1);
%Xref_list=get_Xref(Ts*1);
Xref_list=zeros(13,1);
t0=0;
for i=1:sim_steps
    t=Ts*(i+p);
    %next step reference state
%    Xref=get_Xref(t);
    Xref=zeros(13,1);
    [x1,x2,x3,x4]=unpack_state(x);
    [X1ref,X2ref,X3ref,X4ref]=unpack_state(Xref);
    
    x1=x1+0.05*randn(4,1);
    x2=x2+0.05*randn(4,1);
    x3=x3+0.05*randn(2,1);
    x4=x4+0.05*randn(2,1);
    unew1=mpc_1.evaluate(x1);%,'x.reference', X1ref);
    unew2=mpc_2.evaluate(x2);%,'x.reference', X2ref);
    unew3=mpc_3.evaluate(x3);%,'x.reference', X3ref);
    unew4=mpc_4.evaluate(x4);%,'x.reference', X4ref);
    unew=[unew1;unew2;unew3;unew4];
    u=invT*(unew-G);
%     x_withg=Ad*x_withg+Bd*u;
%     x=x_withg(1:12,1);
    [t0,x,u0] = sim_nolinearquad(Ts,t0,x,u',f);
    
    x_list=[x_list,x];
    u_list=[u_list,u];
    Xref_list=[Xref_list,Xref(1:13,:)];
end

t=0:Ts:sim_time;
% figure();
% plot(t,x_list(1:3,:)');
% figure();
% plot(t,u_list')
% figure();
% plot(t,Xref_list')

%% discretizing
function [Ad,Bd]=discretize_state_function(A,B,Ts)
    Ad=eye(size(A,1))+Ts*A+1/2*A^2*Ts^2+1/6*A^3*Ts^3;
    Bd=(eye(size(A,1))*Ts+1/2*A*Ts^2+1/6*A^2*Ts^3+1/24*A^3*Ts^4)*B;
end
%% unpack whole state to state of each subsystem
function [x1,x2,x3,x4] = unpack_state(x)
    g=9.81;
    x1=[x(1),x(7),g*x(5),g*x(11)]';
    x2=[x(2),x(8),-g*x(4),-g*x(10)]';
    x3=[x(3),x(9)]';
    x4=[x(6),x(12)]';
end
%% reference trajectory
function X_ref = get_Xref(t)
    a=-0.75;
    b=-0.75;
    c=1.5;
    x = a*sin(2/3*t);
    y = b*cos(2/3*t);
    z = c*cos(1/3*t);
    phi = zeros(1,length(t));
    theta = zeros(1,length(t));
    psi = zeros(1,length(t));
    xdot = zeros(1,length(t));
    ydot = zeros(1,length(t));
    zdot = zeros(1,length(t));
    phidot = zeros(1,length(t));
    thetadot = zeros(1,length(t));
    psidot = zeros(1,length(t));
    g=-9.81*ones(1,length(t));
    X_ref = [x;y;z;phi;theta;psi;xdot;ydot;zdot;phidot;thetadot;psidot;g];
    X_ref=reshape(X_ref,size(t,2)*13,1);
end