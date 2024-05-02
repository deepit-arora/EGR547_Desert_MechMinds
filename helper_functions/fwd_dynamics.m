function [q,qd] = fwd_dynamics(u,dt,q,qd)
taufun= @(robot,T,q,qd) u;
q0=q;
qd0=qd;
T=dt;
[ti,q,qd] = R.fdyn(T,taufun,q0,qd0);
