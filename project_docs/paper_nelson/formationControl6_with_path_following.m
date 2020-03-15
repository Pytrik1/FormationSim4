clc
clear
clf(figure(1))
clf(figure(2))
N=4;
time = [0 300];

%starting points for each agent 1 through 3
p01 = [5; 10];
p02 = [5; 12];
p03 = [2; 10];
p04 = [2; 12];

p0cen = 1/N*(p01+p02+p03+p04)
gamma0 = [0;0];
eta0 = [0; 0; 0; 0; 0; 0; 0; 0];
p0 = [p01; p02; p03; p04; gamma0; eta0];

%pbstacles
obs1 = [3; 7];
obs2 = [1; 4];
%Safe distance and Obstacle radii
Rsafe = 0.5;
Robs = 0.75;

%Desired relative distances
p21_star = [0; 3];
p31_star = [-2; 0];
p41_star = [-2; 3];

%Desired relative angles
a21_star = 0.0

%ode solver
tic
%for iteration = 1:1000
[t, p]=ode23(@formation,time,p0);
%end
toc;

error = zeros(length(p),3);

for i = 1:size(p)
    error(i,1) = norm(p21_star)-norm([p(i,3); p(i, 4)]-[p(i,1); p(i, 2)]);
    error(i,2) = norm(p31_star)-norm([p(i,5); p(i, 6)]-[p(i,1); p(i, 2)]);
    error(i,3) = norm(p41_star)-norm([p(i,7); p(i, 8)]-[p(i,1); p(i, 2)]);
end

%plotting
figure(1)
hold on
plot(p01(1),p01(2), 'bo',...
     p02(1),p02(2), 'ro',...
     p03(1),p03(2), 'go',...
     p04(1),p04(2), 'mo',...
     p0cen(1),p0cen(2), 'black*',...
     p(:,1),p(:,2), 'b',...
     p(:,3),p(:,4), 'r',...
     p(:,5),p(:,6), 'g',...
     p(:,7),p(:,8), 'm',...
     p(56,1),p(56,2), 'b*',...
     p(56,3),p(56,4), 'r*',...
     p(56,5),p(56,6), 'g*',...
     p(56,7),p(56,8), 'm*',...
     (p(end,1)+p(end,3)+p(end,5)+p(end,7))/N,(p(end,2)+p(end,4)+p(end,6)+p(end,8))/N, 'black*',...
     [p01(1) p02(1)], [p01(2) p02(2)], 'black',...
     [p02(1) p03(1)], [p02(2) p03(2)], 'black',...
     [p03(1) p04(1)], [p03(2) p04(2)], 'black',...
     [p01(1) p03(1)], [p01(2) p03(2)], 'black',...
     [p01(1) p04(1)], [p01(2) p04(2)], 'black',...
     [p02(1) p04(1)], [p02(2) p04(2)], 'black',...
     [p(end,1) p(end,3)], [p(end,2) p(end,4)], 'black',...
     [p(end,1) p(end,5)], [p(end,2) p(end,6)], 'black',...
     [p(end,3) p(end,5)], [p(end,4) p(end,6)], 'black',...
     [p(end,1) p(end,7)], [p(end,2) p(end,8)], 'black',...
     [p(end,3) p(end,7)], [p(end,4) p(end,8)], 'black',...
     [p(end,5) p(end,7)], [p(end,6) p(end,8)], 'black');
circle(obs1(1),obs1(2),Robs+Rsafe,'b');
circle(obs1(1),obs1(2),Robs,'r');
circle(obs2(1),obs2(2),Robs+Rsafe,'b');
circle(obs2(1),obs2(2),Robs,'r');

axis([-2 13 -2 13]);
axis equal
xlabel('x')
ylabel('y')
legend('agent1', 'agent2', 'agent3', 'agent4')
hold off
figure(2)
hold on
plot(error(1:350,1), 'r-');
plot(error(1:350,2), 'g-');
plot(error(1:350,3), 'm-');
xlabel('iteration')
ylabel('Error')
legend('error21', 'error31', 'error41')

function dx=formation(t,x)
    %Obstacles
    obs1 = [4; 7];
    obs2 = [1; 4];
    
    goal = [-1; -1];
    
    %Safe distance and Obstacle radii
    Rsafe = 0.5;
    Robs = 0.75;

    %Gains
    cf = 10;
    cP = 4;
    cI = 0.8;
    calpha = 400;
    czeta = 5;
    
    %Desired relative distances
    p21_star = [0; 3];
    p31_star = [-2; 0];
    p41_star = [-2; 3];
 
    p01 = x([1 2]);
    p02 = x([3 4]);
    p03 = x([5 6]);
    p04 = x([7 8]);
    
    u1f = ((p02-p01)-p21_star+(p03-p01-(p31_star)));
    u2f = (((p01-p02)-(-p21_star))+(p04-p02-(p41_star-p21_star)));
    u3f = ((p01-p03)-(-p31_star)+(p04-p03-(p41_star-p31_star)));
    u4f = ((p02-p04-(p21_star-p41_star))+(p03-p04-(p31_star-p41_star)));
    
    Uf = cf*[u1f; u2f; u3f; u4f];
    
    pcen = (1/4)*(p01+p02+p03+p04)-goal;
    gamma = x([9 10]);
    
    u1g = cP*pcen-cI*gamma;
    u2g = cP*pcen-cI*gamma;
    u3g = cP*pcen-cI*gamma;
    u4g = cP*pcen-cI*gamma;
    
    Ug = [u1g; u2g; u3g; u4g];
    
    %Unit vector * gain
    alpha1_1 = (p01-obs1)/(norm(p01-obs1)) * 1/(norm(p01-obs1)-Robs);
    alpha1_2 = (p01-obs2)/(norm(p01-obs2)) * 1/(norm(p01-obs2)-Robs);
    
    alpha2_1 = (p02-obs1)/(norm(p02-obs1)) * 1/(norm(p02-obs1)-Robs);
    alpha2_2 = (p02-obs2)/(norm(p02-obs2)) * 1/(norm(p02-obs2)-Robs);
    
    alpha3_1 = (p03-obs1)/(norm(p03-obs1)) * 1/(norm(p03-obs1)-Robs);
    alpha3_2 = (p03-obs2)/(norm(p03-obs2)) * 1/(norm(p03-obs2)-Robs);
    
    alpha4_1 = (p04-obs1)/(norm(p04-obs1)) * 1/(norm(p04-obs1)-Robs);
    alpha4_2 = (p04-obs2)/(norm(p04-obs2)) * 1/(norm(p04-obs2)-Robs);
    
    agent_to_distance = [
        norm(p01-obs1)-Robs;
        norm(p01-obs2)-Robs;
        norm(p02-obs1)-Robs;
        norm(p02-obs2)-Robs;
        norm(p03-obs1)-Robs;
        norm(p03-obs2)-Robs;
        norm(p04-obs1)-Robs;
        norm(p04-obs2)-Robs];
    
    for i = 1:length(agent_to_distance)
        if agent_to_distance(i) <= Rsafe
            agent_to_distance(i) = 1;
        else
            agent_to_distance(i) = 0;
        end
    end
    
%     for i = 1:length(agent_to_distance)
%         agent_to_distance(i) = exp(-7*agent_to_distance(i));
%     end

    u1o = agent_to_distance(1)*calpha*alpha1_1 + agent_to_distance(2)*calpha*alpha1_2;
    u2o = agent_to_distance(3)*calpha*alpha2_1 + agent_to_distance(4)*calpha*alpha2_2;
    u3o = agent_to_distance(5)*calpha*alpha3_1 + agent_to_distance(6)*calpha*alpha3_2;
    u4o = agent_to_distance(7)*calpha*alpha4_1 + agent_to_distance(8)*calpha*alpha4_2;
    
    zeta1 = x([11; 12]);
    zeta2 = x([13; 14]);
    zeta3 = x([15; 16]);
    zeta4 = x([17; 18]);
    
    zeta = [zeta1; zeta2; zeta3; zeta4]; 
    
    zetadot1 = ((zeta2-zeta1)+(zeta3-zeta1)+(zeta4-zeta1))+u1o;
    zetadot2 = ((zeta1-zeta2)+(zeta3-zeta2)+(zeta4-zeta2))+u2o;
    zetadot3 = ((zeta1-zeta3)+(zeta2-zeta3)+(zeta4-zeta3))+u3o;
    zetadot4 = ((zeta1-zeta4)+(zeta2-zeta4)+(zeta3-zeta4))+u4o;
    
    zetaDot = czeta*[zetadot1; zetadot2; zetadot3; zetadot4];
   
    U0 = zeta; 
    
    dx=[Uf-Ug+U0;-pcen; zetaDot];
    i=i+1;    
end
    



