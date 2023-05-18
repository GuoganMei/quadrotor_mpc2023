close all
figure(1);
subplot(2,2,1);
plot(t,Xref_list(1:3,:)','b','Linewidth',1);
hold on
grid on
scatter(t,x_list(1,:)',2,'g','filled');
scatter(t,x_list(2,:)',2,'r','filled');
scatter(t,x_list(3,:)',2,'m','filled');
xlabel('t/s');
ylabel('position/m');
legend('ref_x','ref_y','ref_z','x','y','z');

subplot(2,2,2);
hold on
grid on
plot(t,x_list(4:6,:)');
xlabel('t/s');
ylabel('orientation/m');
legend('\phi','\theta','\psi');

subplot(2,2,3);
hold on
grid on
plot(t,x_list(7:9,:)');
xlabel('t/s');
ylabel('linear velocity/(m/s)');
legend('dx/dt','dy/dt','dz/dt');

subplot(2,2,4);
hold on
grid on
plot(t,x_list(10:12,:)');
xlabel('t/s');
ylabel('angular velocity/(m/s)');
legend('d\phi/dt','d\theta/dt','d\psi/dt');

figure(2);
hold on
grid on
plot(t,u_list');
xlabel('t/s');
ylabel('control input/(rad^2/s^2)');
legend('u_1','u_2','u_3','u_4');