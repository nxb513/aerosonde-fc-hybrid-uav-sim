function [Pfc, Psc, es] = ems_adapt(pp, s, P_dem, dt)
%EMS_ADAPT Bo loc tan so thich nghi theo nang luong sieu tu [R3] (baseline nominal_adapt).
%   f_c = (fH - fL) exp(-5 SoC_sc) + fL, fL = 0.125 Hz, fH = 1 Hz; khi sieu tu dang nap
%   dung (1 - SoC_sc) [R3]. Khong co sieu tu: f_c = 0.5 Hz co dinh [R3].
%   Phan tach va hieu chinh SoC giong ems_filter (ems_split).
es = s.ems;
if ~isfield(es, 'P_lf'), es.P_lf = P_dem; es.prev_sc = 0; end
if pp.sc.use
    e = (s.v_sc/pp.sc.Vmax)^2;
    socsc = (e - pp.sc.e_min)/(1 - pp.sc.e_min);
    if es.prev_sc < 0, socsc = 1 - socsc; end
    f_c = (1 - 0.125)*exp(-5*min(max(socsc, 0), 1)) + 0.125;
else
    f_c = 0.5;
end
[Pfc, Psc, es] = ems_split(pp, s, P_dem, dt, f_c, es);
es.prev_sc = Psc;
end
