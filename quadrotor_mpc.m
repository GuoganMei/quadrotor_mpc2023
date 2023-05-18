clc;
clear;

FormulateNolinearStatefunction();
load('system.mat');
load('NolinearStatefunction');

g=9.81;
A12=zeros(12,1);
A12(9,1)=1;
A21=zeros(1,12);
A22=0;
B12=zeros(1,4);
A=[A,A12;A21,A22];
B=[B;B12];
C=[zeros(3,3),eye(3),zeros(3,7)];
ymax=[1,1,1]';
ymin=[-1,-1,-1]';
umax=[12,12,12,12]';
umin=[0,0,0,0]';

%sample time
Ts=0.05;
%prediction horizon
p=25;
%state dimension
state_n=size(A,1);
%control dimension
input_n=size(B,2);
%state weighting matrix
Q=diag([500,500,500,100,100,100,1,1,1,1,1,1,1]);
%Q(13,13)=0;
%control weighting matrix
R=eye(input_n)*0.1;

x0=[0,0,0,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,-g]';

%discretizing
Ad=eye(state_n)+Ts*A+1/2*A^2*Ts^2+1/6*A^3*Ts^3;
Bd=(eye(state_n)*Ts+1/2*A*Ts^2+1/6*A^2*Ts^3+1/24*A^3*Ts^4)*B;
Cd=C;

[Aqp,Bqp]=forword_predict(p,Ad,Bd);

x=x0;

x_list=x0;
u_list=zeros(4,1);
Xref_list=get_Xref(0);

temp=repmat({Q},p,1);
Qqp=blkdiag(temp{:});
temp=repmat({R},p,1);
Rqp=blkdiag(temp{:});

sim_time=25;
sim_steps=sim_time/Ts;

t0=0;
for i=1:sim_steps
    %Xref=get_Xref(p,state_n);
    t=Ts*(i+1:i+p);
    %Xref=zeros(size(t,2)*13,1);
    Xref=get_Xref(t);
    %Xref=get_Xref2(t);
    
    x_fake=x;
    x_fake(1:3,:)=x(1:3,:)+0.01*randn(3,1);
    x_fake(4:6,:)=x(4:6,:)+0.01*randn(3,1);
    x_fake(7:9,:)=x(7:9,:)+0.09*randn(3,1);
    x_fake(10:12,:)=x(10:12,:)+0.01*randn(3,1);
    
    [Hqp,fqp]=formulate_qp(Aqp,Bqp,Qqp,Rqp,x_fake,Xref);
    [conA,conb]=formulate_constraint(Aqp,Bqp,Cd,p,input_n,ymax,ymin,umax,umin,x);
    Hqp=(Hqp+Hqp')/2;%to be delete
    u_pred = quadprog(Hqp,fqp,conA,conb);
    
    % sim
    [t0,x(1:12,:),u0] = sim_nolinearquad(Ts,t0,x(1:12,:),u_pred(1:input_n)',f);
    %x=Ad*x+Bd*u_pred(1:input_n);   %sim with linear state function
    x_list=[x_list,x];
    u_list=[u_list,u_pred(1:input_n,1)];
    Xref_list=[Xref_list,Xref(1:13,:)];
end
t=0:Ts:sim_time;

%% calculate p step prediction function
function [Aqp,Bqp]=forword_predict(p,A,B)
    Aqp=zeros(p*size(A,1),size(A,2));
    Bqp=zeros(p*size(B,1),p*size(B,2));
    for i=1:p
        Aqp((i-1)*size(A,1)+1:i*size(A,1),:)=A^i;
    end
    for i=1:p
        for j=1:i
            Bqp((i-1)*size(B,1)+1:i*size(B,1),(j-1)*size(B,2)+1:j*size(B,2))=A^(i-j)*B;
        end
    end
end


%% tracking reference trajectory
function X_ref = get_Xref(t)
    a=-0.75;
    b=-0.75;
    c=1.5;
    x = a*sin(2/3*t);
    y = b*cos(2/3*t)-b;
    z = c*cos(1/3*t)-c;
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

function X_ref=get_Xref2(t)
a=0.5;
x_change1=5;
x_change2=15;
b=0.5;
y_change=10;
c=0.25;
z_change=10;
x=zeros(size(t));
y=zeros(size(t));
z=zeros(size(t));
item=1;
for i=t
    if i<=x_change1
       x(item)=a*i;
    elseif i<=x_change2
       x(item)=a*x_change1-a*(i-x_change1);
    else
       x(item)=a*x_change1-a*(x_change2-x_change1)+a*(i-x_change2);
    end
    
    if i<=y_change
        y(item)=b*i;
    else
        y(item)=b*y_change-b*(i-y_change);
    end   
    
    if i<=z_change
        z(item)=c*i;
    else
        z(item)=c*z_change-c*(i-z_change);
    end
    item=item+1;
end
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
%% formulate QP problem
function [H,f]=formulate_qp(Aqp,Bqp,Q,R,x0,X_ref)
    A=Bqp;
    B=Aqp*x0-X_ref;
    H=2*(A'*Q*A+R);
    f=A'*(Q+Q')*B;
end
%% formulate state constraint and control constraint
function [A,b]=formulate_constraint(Aqp,Bqp,C,p,input_n,ymax,ymin,umax,umin,x)
    Ymax=repmat(ymax,p,1);
    Ymin=repmat(ymin,p,1);
    Umax=repmat(umax,p,1);
    Umin=repmat(umin,p,1);
    
    temp=repmat({C},p,1);
    Cqp=blkdiag(temp{:});
    Iu=eye(input_n*p);
    
    A1=Cqp*Bqp;
    b1=Ymax-Cqp*Aqp*x;
    A2=-Cqp*Bqp;
    b2=Cqp*Aqp*x-Ymin;
    A3=Iu;
    b3=Umax;
    A4=-Iu;
    b4=-Umin;
    
    A=[A1;A2;A3;A4];
    b=[b1;b2;b3;b4];
end