function [t,x]=H4_PlatoonRobots()

clc
close all


time=[0 300];
x0=[0;0;0;0;0;0;0;0;0;0;0;0];

[t, x]=ode23(@vehicles,time,x0);
plot(x(:,1),x(:,3),'b',...
    x(:,5),x(:,7),'r',...
    x(:,9),x(:,11),'g',...
    Goal(1),Goal(2),'o',...
    Obs1(1),Obs1(2),'x',...
    Obs2(1),Obs2(2),'x')
hold on
p1=plot(x(1,1),x(1,3),'o','MarkerFaceColor','blue');
p2=plot(x(1,5),x(1,7),'o','MarkerFaceColor','red');
p3=plot(x(1,9),x(1,11),'o','MarkerFaceColor','green');
axis([0 12 0 12])

% for k = 2:length(x)
%     p1.XData=x(k,1);
%     p1.YData=x(k,3);
%     p2.XData=x(k,5);
%     p2.YData=x(k,7);
%     p3.XData=x(k,9);
%     p3.YData=x(k,11);
%     %pause(0.00001)
%     drawnow 
% end

    function dx=vehicles(t,x)
        Goal=[10;10];
        Obs1=[0;3];
        Obs2=[0;4];
        KG=20;
        Ko=30;
        Kij=20;
        KLi=100;
        KF=10;
        rD=0.5; %Separation from the leader

        rG=sqrt((Goal(1)-x(1))^2+(Goal(2)-x(3))^2); %Leader and goal
        FGx=KG*(Goal(1)-x(1))/max([rG 0.01]);    %Avoid division by zero
        FGy=KG*(Goal(2)-x(3))/max([rG 0.01]);

        ro1=sqrt((Obs1(1)-x(1))^2+(Obs1(2)-x(3))^2);    %Leader and Obstacle1
        Fo1x=-Ko*(Obs1(1)-x(1))/ro1^3;
        Fo1y=-Ko*(Obs1(2)-x(3))/ro1^3;

        ro2=sqrt((Obs2(1)-x(1))^2+(Obs1(2)-x(3))^2);    %Leader and obstacle2
        Fo2x=-Ko*(Obs2(1)-x(1))/ro2^3;
        Fo2y=-Ko*(Obs2(2)-x(3))/ro2^3;

        FrLx=-KF*x(2);
        FrLy=-KF*x(4);

        Fx=(FGx+Fo1x+Fo2x+FrLx);
        Fy=(FGy+Fo1y+Fo2y+FrLy);

        xL=[0 1;0 0]*[x(1);x(2)]+[0;1]*Fx;  %Leader dynamics
        yL=[0 1;0 0]*[x(3);x(4)]+[0;1]*Fy;

        r1o1=sqrt((Obs1(1)-x(5))^2+(Obs1(2)-x(7))^2);   %Follower1 and Obs1
        F1o1x=-Ko*(Obs1(1)-x(5))/r1o1^3;
        F1o1y=-Ko*(Obs1(2)-x(7))/r1o1^3;

        r1o2=sqrt((Obs2(1)-x(5))^2+(Obs2(2)-x(7))^2);   %Follower1 and Obs2
        F1o2x=-Ko*(Obs2(1)-x(5))/r1o2^3;
        F1o2y=-Ko*(Obs2(2)-x(7))/r1o2^3;

        rL1=sqrt((x(1)-x(5))^2+(x(11)-x(7))^2); %Follower1 and Leader
        FL1x=-2*KLi*(rL1-rD)*(x(1)-x(5))/max([rL1^4 0.01]);
        FL1y=-2*KLi*(rL1-rD)*(x(3)-x(7))/max([rL1^4 0.01]);

        r21=sqrt((x(9)-x(5))^2+(x(11)-x(7))^2); %Follower1 and Follower2
        F21x=-2*Kij*(x(9)-x(5))/max([r21^4 0.01]);
        F21y=-2*Kij*(x(11)-x(7))/max([r21^4 0.01]);

        Fr1x=-KF*x(6);
        Fr1y=-KF*x(8);

        F1x=(F1o1x+F1o2x+FL1x+F21x+Fr1x);
        F1y=(F1o1y+F1o2y+FL1y+F21y+Fr1y);

        x1=[0 1;0 0]*[x(5);x(6)]+[0;1]*F1x; %Follower1 dynamics
        y1=[0 1;0 0]*[x(7);x(8)]+[0;1]*F1y;

        r2o1=sqrt((Obs1(1)-x(9))^2+(Obs1(2)-x(11))^2);   %Follower2 and Obs1
        F2o1x=-Ko*(Obs1(1)-x(9))/r2o1^3;
        F2o1y=-Ko*(Obs1(2)-x(11))/r2o1^3;

        r2o2=sqrt((Obs2(1)-x(9))^2+(Obs2(2)-x(11))^2);   %Follower2 and Obs2
        F2o2x=-Ko*(Obs2(1)-x(9))/r2o2^3;
        F2o2y=-Ko*(Obs2(2)-x(11))/r2o2^3;

        rL2=sqrt((x(1)-x(9))^2+(x(3)-x(11))^2); %Follower2 and Leader
        FL2x=-2*KLi*(rL2-rD)*(x(1)-x(9))/max([rL2^4 0.01]);
        FL2y=-2*KLi*(rL2-rD)*(x(3)-x(11))/max([rL2^4 0.01]);

        r12=sqrt((x(5)-x(9))^2+(x(7)-x(11))^2); %Follower2 and Follower1
        F12x=-2*Kij*(x(5)-x(9))/max([r12^4 0.01]);
        F12y=-2*Kij*(x(7)-x(11))/max([r12^4 0.01]);

        Fr2x=-KF*x(10);
        Fr2y=-KF*x(12);

        F2x=(F2o1x+F2o2x+FL2x+F12x+Fr2x);
        F2y=(F2o1y+F2o2y+FL2y+F12y+Fr2y);

        x2=[0 1;0 0]*[x(9);x(10)]+[0;1]*F2x; %Follower2 dynamics
        y2=[0 1;0 0]*[x(11);x(12)]+[0;1]*F2y;

        dx=[xL(1);xL(2);yL(1);yL(2);x1(1);x1(2);y1(1);y1(2);x2(1);x2(2);...
            y2(1);y2(2)];

        if x(2)==0 && x(4)==0   %Avoid local minima
            w=0.001*randn(2,1);
            dx=dx+[0;w(1);0;w(2);0;0;0;0;0;0;0;0];
        end

        if r12==0   %Separate followers from initial condition
            w=0.001*randn(4,1);
            dx=dx+[0;0;0;0;0;w(1);0;0;w(2);w(3);0;w(4)];
        end
        
    end
end
    
    