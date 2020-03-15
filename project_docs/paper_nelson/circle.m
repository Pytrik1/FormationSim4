function [xp, yp] = circle(x,y,r,c)
%x and y are the coordinates of the center of the circle
%r is the radius of the circle
%0.01 is the angle step, bigger values will draw the circle faster but
%you might notice imperfections (not very smooth)
ang=0:0.01:2*pi; 
xp=r*cos(ang)+x;
yp=r*sin(ang)+y;
%plot(x+xp,y+yp);
fill(xp, yp,c)
end