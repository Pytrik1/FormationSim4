close all
clear
clc

xf=0:0.1:12;    %%Creates linear spaced range
yf=0:0.1:12;    %%--
[X, Y]=meshgrid(xf,yf); %% returns 2-D grid based on the length of xf and yf

%%Coordinates
Goal=[10;10];
Obs1=[2;2];
Obs2=[9;9];
Obs3=[3;3];
KG=10;      %%gains
Ko=5;      %%

rG=sqrt((Goal(1)-X).^2+(Goal(2)-Y).^2);
VG=KG*rG; %Potential field for the goal

ro1=sqrt((Obs1(1)-X).^2+(Obs1(2)-Y).^2);
Vo1=Ko./ro1; %Potential field for the first obstacle

ro2=sqrt((Obs2(1)-X).^2+(Obs2(2)-Y).^2);
Vo2=Ko./ro2; %Potential field for the second obstacle

ro3=sqrt((Obs3(1)-X).^2+(Obs3(2)-Y).^2);
Vo3=Ko./ro3; %Potential field for the third obstacle

f1=figure;
mesh(xf,yf,VG+Vo1+Vo2+Vo3)
xlabel('x')
ylabel('y')
zlabel('V')
f2=figure;
[t,x]=H4_PotentialNavigation();


