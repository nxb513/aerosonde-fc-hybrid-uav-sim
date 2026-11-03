function [dx, aux] = ac_derivatives(x, cmd, Ub, w_ned, g_b, p, Vcap)
%AC_DERIVATIVES Dao ham trang thai 6-DOF + servo (Aerosonde).
%   cmd = [de_c; da_c; dr_c; dt_c] (da qua tre); Vcap tran dien ap dong co (tuy chon).
%   Than cung [R37, Eq. 3.13-3.16]: dv = F/m - om x v, dom = J^-1 (M - om x J om),
%   dpos = R_nb v; tu the bang quaternion [R37, App. B].
if nargin < 7, Vcap = []; end
I = ac_state_index();
v = x(I.vel); q = x(I.q)/norm(x(I.q)); om = x(I.om);
[F, M, aux] = ac_forces_moments(v, q, om, x(I.dlt), x(I.thr), Ub, w_ned, g_b, p, Vcap);
[d_dot, dd_dot, th_dot] = ac_actuator_deriv(x(I.dlt), x(I.ddlt), x(I.thr), cmd, p);
dx = zeros(I.n, 1);
dx(I.pos)  = quat_utils('dcm', q)*v;
dx(I.vel)  = F/p.mass.m - cross(om, v);
dx(I.q)    = quat_utils('deriv', q, om);
dx(I.om)   = p.mass.Jinv*(M - cross(om, p.mass.J*om));
dx(I.dlt)  = d_dot;
dx(I.ddlt) = dd_dot;
dx(I.thr)  = th_dot;
aux.F = F; aux.M = M;
end
