function [dx, aux] = x8_derivatives(x, cmd, Ub, w_ned, g_b, p)
%X8_DERIVATIVES Dao ham trang thai X8 6-DOF + canh quat + servo.
%   [dx, aux] = X8_DERIVATIVES(x, cmd, Ub, w_ned, g_b, p)
%     x    trang thai 19 chieu (xem x8_state_index)
%     cmd  lenh da qua tre [der_c; del_c; dt_c]
%
%   Than cung [R2, Eq. 23-26][R37, Eq. 3.14, 3.16]:
%     dv = F/m - om x v,  dom = J^-1 (M - om x J om),  dpos = R_nb v
%   Tu the bang quaternion [R37, App. B].
%   Servo elevon bac 2 (wn, zeta) [R2, Bang 9] co bao hoa bien do/toc do [R7];
%   ga bac 1 [R2, Bang 9]. Tre van chuyen xu ly trong x8_simulate.

I  = x8_state_index();
v  = x(I.vel);
q  = x(I.q);  q = q/norm(q);
om = x(I.om);
Om = x(I.Om);
d  = x(I.dlt);
dd = x(I.ddlt);
th = x(I.thr);

[F, M, aux] = x8_forces_moments(v, q, om, Om, d(1), d(2), th, Ub, w_ned, g_b, p);

m = p.mass.m; J = p.mass.J;
dv  = F/m - cross(om, v);
dom = p.mass.Jinv*(M - cross(om, J*om));
dpos = quat_utils('dcm', q)*v;
dq  = quat_utils('deriv', q, om);

[dd_out, ddd, dth] = x8_actuator_deriv(d, dd, th, cmd, p);

dx = zeros(I.n, 1);
dx(I.pos)  = dpos;
dx(I.vel)  = dv;
dx(I.q)    = dq;
dx(I.om)   = dom;
dx(I.Om)   = aux.prop.dOm;
dx(I.dlt)  = dd_out;
dx(I.ddlt) = ddd;
dx(I.thr)  = dth;
aux.F = F; aux.M = M;
end
