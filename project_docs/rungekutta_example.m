clear all
close all
clc

h=0.1;
x=[0 300];
y=zeros(1,length(x));
y0=[0;0;0];
n=3000;

% y_dot=@(x,y)(0)

y_dot=vehicle(x,y0)

for i=1:n
    k1=y_dot(x(i),y(i));
    k2=y_dot(x(i)+.5*h,y(i)+0.5*k1*h);
    k3=y_dot(x(i)+.5*h,y(i)+0.5*k2*h);
    k4=y_dot(x(i)+h,y(i)+k3*h);
    y(i+1)=y(i)+((k1+2*k2+2*k3+k4)/6)*h;
end

[t, y_check]=ode23(y_dot,x,y0);

plot(x,y)
title('RungeKutta')
figure
plot(x,y_check)
title('ode23 Check')

    function dx=vehicle(t,x)
        
        Goal=[10;10];
        Obs1=[3;3];
        Obs2=[8;8];
        Obs3=[6;2];
        KG=30;
        Ko=30;
        
        rG=sqrt((Goal(1)-x(1))^2+(Goal(2)-x(2))^2);
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