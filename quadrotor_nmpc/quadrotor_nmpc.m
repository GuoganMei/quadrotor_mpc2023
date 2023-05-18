%% NMPC based Quadrotor trajectory tracking
clear all;
close all;
clc;
%% define NMPC problem
NMPC_problem_formulation;
load NMPC_problem_definition.mat
%% control simulation    

% timestep
dt = T;
t0 = 0;

x0 = [0;0;4;0;0;0;0;0;0;0;0;0];
% record states
xx(:,1) = x0;
u0 = zeros(N,length_control);
X0 = repmat(x0,1,N+1)';

% max simulation time
sim_tim = 23; 

u_c1=[];
x_list=[];
sim_steps=sim_tim/dt;

x0_fake=x0;
for i=1:sim_steps
    current_time = t0;
    % init state
    args.p(1:length_state,1) = x0_fake;%x0;
    % reference
    t_predict = current_time:T:current_time+T*(N-1);
    referencestate = QuadrotorReferenceTrajectory(t_predict);
    referencecontrol = 4.9*ones(4,N);
    reference = [referencestate;referencecontrol];
    args.p(length_state_control-length_control+1:length_state_control*N+length_state,1) = reshape(reference,length_state_control*N,1);

    % initialization value of OPT variables
    args.x0 = [reshape(X0',length_state*(N+1),1);reshape(u0',length_control*N,1)];

    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
        'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    % xx1(:,1:length_state,nmpciter+1) = reshape(full(sol.x(1:length_state*(N+1)))',length_state,N+1)';
    u = reshape(full(sol.x(length_state*(N+1)+1:end))',length_control,N)';
    u_c1 = [u_c1;u(1,:)];
    [t0, x0, u0] = sim_nolinearquad(dt,t0,x0,u,f);
    x0_fake=x0+0.5*randn(12,1);
    x_list=[x_list,x0];
end

t_allref = 0:dt:sim_tim;
Xref_list = QuadrotorReferenceTrajectory(t_allref);
t=dt:dt:sim_tim;
Xref_list=Xref_list(:,1:end-1);
u_list=u_c1;
% figure();
% plot(x_list');