function dydt = vdp1(t,y)
%vdp1 Evaluate the van der Pol ODEs for mu =1
%
%  See also ODE113, ODE23, ODE45/
%

dydt = [y(2); A/b*t*y(1)];