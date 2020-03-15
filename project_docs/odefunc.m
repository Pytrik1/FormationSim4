function dydt = odefunc(t,y,A,B)
%vdp1 Evaluate the van der Pol ODEs for mu =1
%
%  See also ODE113, ODE23, ODE45/
%

dydt = [y(2); A/B*t*y(1)];