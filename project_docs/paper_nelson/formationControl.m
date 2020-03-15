close all
clc
clear

time = [0 300];




p21_star = [0; 3];
p31_star = [-2; 0];
p01 = [5; 10];
p02 = [5; 12];
p03 = [2; 10];
p0cen = 1/3*(p01+p02+p03);
p0 = [p01; p02; p03; p0cen];


[t, p]=ode23(@formation,time,p0);

obs1 = [4; 7];
obs2 = [1; 4];

%Safe distance and Obstacle radii
Rsafe = 0.5
Robs = 0.75

figure
hold on
plot(p01(1),p01(2), 'o',...
     p02(1),p02(2), 'o',...
     p03(1),p03(2), 'o',...
     p0cen(1),p0cen(2), '*',...
     p(:,1),p(:,2), 'b',...
     p(:,3),p(:,4), 'r',...
     p(:,5),p(:,6), 'y',...
     p(end,7),p(end,8), '*',...
     [p01(1) p02(1)], [p01(2) p02(2)], 'black',...
     [p02(1) p03(1)], [p02(2) p03(2)], 'black',...
     [p01(1) p03(1)], [p01(2) p03(2)], 'black',...
     [p(end,1) p(end,3)], [p(end,2) p(end,4)], 'black',...
     [p(end,1) p(end,5)], [p(end,2) p(end,6)], 'black',...
     [p(end,3) p(end,5)], [p(end,4) p(end,6)], 'black')
circle(obs1(1),obs1(2),Robs+Rsafe,'b');
circle(obs1(1),obs1(2),Robs,'r');
circle(obs2(1),obs2(2),Robs+Rsafe,'b');
circle(obs2(1),obs2(2),Robs,'r');

axis([-2 13 -2 13]);
axis equal
xlabel('x')
ylabel('y')
legend('agent1', 'agent2', 'agent3')

L = [2 -1 -1; -1 2 -1; -1 -1 2]
%Uf = cf*kron(L,eye(2))*(pstar-[p1;p2;p3])

function dx=formation(t,x)
    %gains
    cf = 10;
    cP = 1;
    cI = 0.9;
    calpha = 400;
    csigma = 10;

    p21_star = [0; 3];
    p31_star = [-2; 0];
 
    p01 = x([1 2])
    p02 = x([3 4]);
    p03 = x([5 6]);
    
    gamma = x([7 8]);
    
    u1f = cf*((p02-p01)-p21_star+(p03-p01-(p31_star)));
    u2f = cf*(((p01-p02)-(-p21_star))+(p03-p02-(p31_star-p21_star)));
    u3f = cf*((p01-p03)-(-p31_star)+(p02-p03-(p21_star-p31_star))); 
    
    p_cen = (1/3)*(p01+p02+p03)
    
    %[t, gamma] = ode23(-p_cen, [0 300], [0;0]);
    ug = -cP*p_cen+cI*gamma;
    
    Uf = [u1f; u2f; u3f];
    dx=[Uf;ug];
        
    end



