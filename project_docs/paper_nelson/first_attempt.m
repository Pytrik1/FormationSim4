close all
clc

time = [0 300];




p21_star = [0; 3];
p31_star = [-2; 0];
p01 = [5; 10];
p02 = [5; 12];
p03 = [2; 10];
p0 = [p01; p02; p03];

[t, p]=ode23(@formation,time,p0);

obs1 = [4; 7];
obs2 = [1; 4];

%Safe distance and Obstacle radii
Rsafe = 0.5
Robs = 0.75

%gains
cf = 10;
cp = 1;
ci = 0.9;
calpha = 400;
csigma = 10;

figure
hold on
plot(p01(1),p01(2), 'o',...
     p02(1),p02(2), 'o',...
     p03(1),p03(2), 'o',...
     p(:,1),p(:,2), 'b',...
     p(:,3),p(:,4), 'r',...
     p(:,5),p(:,6), 'y',...
     [p01(1) p02(1)], [p01(2) p02(2)], 'black',...
     [p02(1) p03(1)], [p02(2) p03(2)], 'black',...
     [p01(1) p03(1)], [p01(2) p03(2)], 'black')
circle(obs1(1),obs1(2),Robs+Rsafe,'b');
circle(obs1(1),obs1(2),Robs,'r');
circle(obs2(1),obs2(2),Robs+Rsafe,'b');
circle(obs2(1),obs2(2),Robs,'r');

axis([-2 12 -2 12]);
%axis equal
xlabel('x')
ylabel('y')
legend('agent1', 'agent2', 'agent3')

L = [2 -1 -1; -1 2 -1; -1 -1 2]
%Uf = cf*kron(L,eye(2))*(pstar-[p1;p2;p3])

function dx=formation(t,x)
    p21_star = [0; 3];
    p31_star = [-2; 0];
    cf=1;
    p01 = x([1 2]);
    p02 = x([3 4]);
    p03 = x([5 6]);
    u1f = cf*((p02-p01)-p21_star+(p03-p01-(p31_star))); %looks correct
    u2f = cf*(((p01-p02)-(-p21_star))+(p03-p02-(p31_star-p21_star))); %looks correct
    u3f = cf*((p01-p03)-(-p31_star)+(p02-p03-(p21_star-p31_star))); %looks correct
    v=1; L=2;
    if u1f+u2f+u3f<0.05 %Stop the vehicle when arrives to the goal
        v=0;
    end
    Uf = [u1f; u2f; u3f];
    dx=[v*u1f;v*u2f;v*u3f];
        
    end



