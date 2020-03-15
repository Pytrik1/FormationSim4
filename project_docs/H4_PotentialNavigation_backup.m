function [t,x]=H4_PotentialNavigation_backup()
clear
clc
close all

time=[0 300];
x0=[0;0;0];

[t, x]=ode23(@vehicle,time,x0); %

plot(x(:,1),x(:,2),'r', ...
    Goal(1),Goal(2),'o', ...
    Obs1(1),Obs1(2),'x', ... 
    Obs2(1),Obs2(2),'x', ...
    Obs3(1),Obs3(2),'x')
hold on
p=plot(x(1,1),x(1,2),'o','MarkerFaceColor','red');
hold off
axis([0 12 0 12])

for k=2:length(x)
    p.XData=x(k,1);
    p.YData=x(k,2);
    pause(0.025)
    drawnow 
end

    function dx=vehicle(t,x)
        
        Goal=[10;10];
        Obs1=[3;3];
        Obs2=[8;8];
        Obs3=[6;2];
        KG=30;
        Ko=30;
 
        rG=sqrt((Goal(1)-x(1))^2+(Goal(2)-x(2))^2)
        FGx=KG*(Goal(1)-x(1))/rG;
        FGy=KG*(Goal(2)-x(2))/rG;
        
        ro1=sqrt((Obs1(1)-x(1))^2+(Obs1(2)-x(2))^2);
        Fo1x=-Ko*(Obs1(1)-x(1))/ro1^3;
        Fo1y=-Ko*(Obs1(2)-x(2))/ro1^3;
        
        ro2=sqrt((Obs2(1)-x(1))^2+(Obs2(2)-x(2))^2);
        Fo2x=-Ko*(Obs2(1)-x(1))/ro2^3;
        Fo2y=-Ko*(Obs2(2)-x(2))/ro2^3;
        
        ro3=sqrt((Obs3(1)-x(1))^2+(Obs3(2)-x(2))^2);
        Fo3x=-Ko*(Obs3(1)-x(1))/ro3^3;
        Fo3y=-Ko*(Obs3(2)-x(2))/ro3^3;
        
        Fx=(FGx+Fo1x+Fo2x+Fo3x);
        Fy=(FGy+Fo1y+Fo2y+Fo3y);
        
        alpha=atan(Fy/Fx);
        
        v=1; L=2;
        if rG<0.05 %Stop the vehicle when arrives to the goal
            v=0;
        end
        K=2;
        ph=K*(alpha-x(3));
        dx=[v*cos(ph)*cos(x(3));v*cos(ph)*sin(x(3));v*sin(ph)/L];
        
    end
end
        