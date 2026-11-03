function [F, M, aux] = ac_forces_moments(v_b, q, om_b, dl, dt, Ub, w_ned, g_b, p, Vcap)
%AC_FORCES_MOMENTS Tong luc/momen quanh CG thuc te, he than.
%   [F, M, aux] = AC_FORCES_MOMENTS(v_b, q, om_b, dl, dt, Ub, w_ned, g_b, p, Vcap)
%     dl = [de; da; dr] goc lai thuc te; dt ga thuc te; Ub dien ap bus;
%     w_ned gio deu (NED), g_b gust (he than); Vcap tran dien ap dong co (ac_propulsion).
%   Van toc khi tuong doi [R37, Eq. 2.6-2.7]; luc day doc truc x, momen canh
%   quat -Q quanh x [R37, Eq. 4.26]. CG lech goc mot doan r_cg -> momen quanh CG
%   moi: M_cg = M_O + (r_O - r_cg) x (F_aero + F_prop) (co hoc vat ran).
%   GT: duong luc day di qua goc he than; van toc khi tai CG ~ tai goc.
R_nb = quat_utils('dcm', q);
vr_b = v_b - R_nb.'*w_ned - g_b;
Va = max(norm(vr_b), p.aero.Va_min);
if nargin < 10, Vcap = []; end
pr = ac_propulsion(Va, dt, Ub, p, Vcap);
ae = ac_aero(vr_b, om_b, dl, p);
F_prop = [pr.T; 0; 0];
M_prop = [-pr.Q; 0; 0];
F_ap = ae.F + F_prop;
M = ae.M + M_prop + cross(-p.mass.r_cg, F_ap);
F = F_ap + R_nb.'*[0; 0; p.mass.m*p.env.g];
aux.Va = ae.Va; aux.alpha = ae.alpha; aux.beta = ae.beta;
aux.prop = pr; aux.CL = ae.CL; aux.CD = ae.CD; aux.F_aero = ae.F;
end
