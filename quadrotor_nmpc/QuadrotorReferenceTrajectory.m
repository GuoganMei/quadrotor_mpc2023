function [ xdesired ] = QuadrotorReferenceTrajectory( t )
a=5;
b=-5;
c=3.5;
x = a*sin(2/3*t)-5;
y = b*cos(2/3*t);
z = c*cos(1/3*t)+2;
phi = zeros(1,length(t));
theta = zeros(1,length(t));
psi = zeros(1,length(t));
xdot = zeros(1,length(t));
ydot = zeros(1,length(t));
zdot = zeros(1,length(t));
phidot = zeros(1,length(t));
thetadot = zeros(1,length(t));
psidot = zeros(1,length(t));
xdesired = [x;y;z;phi;theta;psi;xdot;ydot;zdot;phidot;thetadot;psidot];
end

% function [ xdesired ] = QuadrotorReferenceTrajectory( t )
% a=1;
% x_change1=5;
% x_change2=15;
% b=1;
% y_change=10;
% c=0.5;
% z_change=10;
% x=zeros(size(t));
% y=zeros(size(t));
% z=zeros(size(t));
% 
% item=1;
% for i=t
%     if i<=x_change1
%        x(item)=a*i;
%     elseif i<=x_change2
%        x(item)=a*x_change1-a*(i-x_change1);
%     else
%        x(item)=a*x_change1-a*(x_change2-x_change1)+a*(i-x_change2);
%     end
%     
%     if i<=y_change
%         y(item)=b*i;
%     else
%         y(item)=b*y_change-b*(i-y_change);
%     end   
%     
%     if i<=z_change
%         z(item)=c*i;
%     else
%         z(item)=c*z_change-c*(i-z_change);
%     end
%     item=item+1;
% end
% z=z+4;
% 
% phi = zeros(1,length(t));
% theta = zeros(1,length(t));
% psi = zeros(1,length(t));
% xdot = zeros(1,length(t));
% ydot = zeros(1,length(t));
% zdot = zeros(1,length(t));
% phidot = zeros(1,length(t));
% thetadot = zeros(1,length(t));
% psidot = zeros(1,length(t));
% xdesired = [x;y;z;phi;theta;psi;xdot;ydot;zdot;phidot;thetadot;psidot];
% end

