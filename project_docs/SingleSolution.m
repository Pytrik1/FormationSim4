A = 1;
B = 2;
tspan = [0 5];
y0 = [0; 0.01];

[t, y] = ode23(@(t,y) odefunc(t,y,A,B),tspan,y0);

plot(t,y(:,1),t,y(:,2))
legend('y1','y2')