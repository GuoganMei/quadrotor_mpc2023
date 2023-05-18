figure(1);
make_gif(x_list,Xref_list,N,sim_steps,T)
function make_gif(x_list,Xref_list,p,sim_steps,Ts)
    pic_num = 1;
    for i=1:sim_steps-p
        quad_animate(1,x_list,Xref_list,i,p,0.25,[])

        frame = getframe(figure(1));
        [A,map]=rgb2ind(frame2im(frame),256);
        if pic_num==1
            imwrite(A,map,'testAnimated.gif','gif','LoopCount',Inf,'DelayTime',Ts);
        else
            imwrite(A,map,'testAnimated.gif','gif','WriteMode','append','DelayTime',Ts);
        end
        pic_num=pic_num+1;
        % pause(Ts);
    end
end
%% plot quadrotor with R posture
function plot_quadrotor(l,pos,R)
    x = [1, 0, -1,  0,  0,  0] * l;
    y = [0, 1,  0, -1,  0,  0] * l;
    z = [0, 0,  0,  0,  0,  1] * l/6;
    P = R * [x; y; z]+pos;
    order = [1,3,6,2,4,6,1];
    plot3(P(1, order), P(2, order), P(3, order), 'black');
    hold on;
    plot_circle3d(l,pos+R*[l,0,0]',R);
    plot_circle3d(l,pos+R*[-l,0,0]',R);
    plot_circle3d(l,pos+R*[0,l,0]',R);
    plot_circle3d(l,pos+R*[0,-l,0]',R);
end
%% plot circle with R posture
function plot_circle3d(l,pos,R)
    x = cos(linspace(0,2*pi,20))* l/5;
    y = sin(linspace(0,2*pi,20)) * l/5;
    z = zeros(1,20);
    P = R * [x; y; z]+pos;
    plot3(P(1,:),P(2,:),P(3,:),'black');
end
%% plot the past trajectory ,reference trajectory and quadrotor now
function quad_animate(fig,X_list,X_ref_list,sim_step,p,l,View)
    %   X:需要绘制的状态向量
    %   fig:窗口序号
    %   X_list:实际轨迹
    %   X_ref_list:参考轨迹

    figure(fig);
    clf; grid on;axis equal; hold on;
    if ~isempty(View)
        view(View(1), View(2))
    else
        view(-45, 45);%视角控制，目前为跟随机器人后方，从 45 方向看机器人 X(3)*180/pi
    end
    %plot reference trajectory
    if ~isempty(X_ref_list)
        plot3(X_ref_list(1,1:sim_step+p),X_ref_list(2,1:sim_step+p),X_ref_list(3,1:sim_step+p),'b');
    end
    hold on
    %%plot past trajectory
    if ~isempty(X_list)
        scatter3(X_list(1,1:sim_step),X_list(2,1:sim_step),X_list(3,1:sim_step),4,'r','filled');
    end
    X=X_list(:,sim_step);
    
    %rotation matrix
    R = rotz(X(4))*roty(X(5))*rotx(X(6));
    %plot quadrotor
    plot_quadrotor(l,X(1:3,1),R);
 
%     view_range=[-1,1,-1,1,-0.5,3.5];
%     
%     axis(view_range);
    xlabel('x');ylabel('y');zlabel('z');
%     drawnow;
    axis equal;
end