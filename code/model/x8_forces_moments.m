function [F, M, aux] = x8_forces_moments(v_b, q, om_b, Om, der, del, dt, Ub, w_ned, g_b, p)
%X8_FORCES_MOMENTS Tong luc/momen quanh CG thuc te, he than.
%   [F, M, aux] = X8_FORCES_MOMENTS(v_b, q, om_b, Om, der, del, dt, Ub, w_ned, g_b, p)
%     v_b    van toc dat cua CG trong he than [u; v; w]
%     q      quaternion than -> NED
%     om_b   [p; q; r]
%     Om     toc do canh quat
%     der,del goc elevon thuc te (sau servo), dt ga thuc te
%     Ub     dien ap bus DC
%     w_ned  gio deu trong NED, g_b gust trong he than (m/s)
%
%   Tong hop [R2, Eq. 11-13, 15-22]:
%     F = F_aero + F_prop + F_grav,  M = M_aero + M_prop
%   Canh quat day sau: M_prop = -([Q; 0; 0] + [Ip dOm; Ip Om r; -Ip Om q]) [R2, Eq. 11-13].
%   CG lech goc he than mot doan r_cg: momen quanh CG moi
%     M_cg = M_O + (r_O - r_cg) x (F_aero + F_prop)  [R1, Eq. 18]
%   GT: duong luc day di qua goc he than (truc x), nhu [R2].
%   GT: van toc khi tai diem tham chieu ~ van toc khi tai CG (bo qua om x r).

R_nb = quat_utils('dcm', q);
vr_b = v_b - R_nb.'*w_ned - g_b;
Va   = max(norm(vr_b), p.aero.Va_min);

pr = x8_propulsion(Va, Om, dt, Ub, p);
[de, da] = x8_elevon_mix(der, del);
ae = x8_aero(vr_b, om_b, de, da, pr.CT, p);

Ip = p.prop.Ip;
F_prop = [pr.T; 0; 0];
M_prop = -([pr.Q; 0; 0] + [Ip*pr.dOm; Ip*Om*om_b(3); -Ip*Om*om_b(2)]);

F_grav = R_nb.'*[0; 0; p.mass.m*p.env.g];

r_O_from_cg = -p.mass.r_cg;
F_ap = ae.F + F_prop;
M = ae.M + M_prop + cross(r_O_from_cg, F_ap);
F = F_ap + F_grav;

aux.Va = ae.Va; aux.alpha = ae.alpha; aux.beta = ae.beta;
aux.de = de; aux.da = da;
aux.prop = pr; aux.C = ae.C;
aux.F_aero = ae.F; aux.F_prop = F_prop;
end
