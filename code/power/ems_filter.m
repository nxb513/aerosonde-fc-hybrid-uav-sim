function [Pfc, Psc, es] = ems_filter(pp, s, P_dem, dt)
%EMS_FILTER Chien luoc tach tan so [R3] (EMS mac dinh, cung la baseline).
%   FC nhan thanh phan tan so thap LPF(P_dem, f_c); sieu tu nhan phan tan so cao;
%   pin bu phan con lai (ke ca phan FC khong theo kip do gioi han toc do).
%   f_c thay doi theo trang thai sieu tu [R3]: f_c = (fH - fL) exp(-5 SoC) + fL,
%   fL = 0.125 Hz, fH = 1 Hz; khi sieu tu dang nap dung (1 - SoC). Khong co sieu tu:
%   f_c = 0.5 Hz co dinh [R3].
%   Bo sung (GT): hieu chinh SoC pin ve soc_ref va nang luong sieu tu ve 0.5.
%   Muc bu do planner E3 chon: s.ems.fc_fix (Hz) dat tan so cat co dinh thay cho
%   luat thich nghi; f_c thap -> pin/sieu tu ganh nhieu hon, FC muot hon.
es = s.ems;
if ~isfield(es, 'P_lf'), es.P_lf = P_dem; es.prev_sc = 0; end
k_soc = 2.0; soc_ref = s.soc0;
k_e   = 0.5;
soc = 1 - s.it/pp.bat.cell.Q;
if isfield(es, 'fc_fix') && ~isempty(es.fc_fix)
    fc = es.fc_fix;
elseif pp.sc.use
    e = (s.v_sc/pp.sc.Vmax)^2;
    socsc = (e - pp.sc.e_min)/(1 - pp.sc.e_min);
    if es.prev_sc < 0, socsc = 1 - socsc; end
    fc = (1 - 0.125)*exp(-5*min(max(socsc, 0), 1)) + 0.125;
else
    fc = 0.5;
end
a = 1 - exp(-2*pi*fc*dt);
es.P_lf = es.P_lf + a*(P_dem - es.P_lf);
Pfc = es.P_lf + k_soc*(soc_ref - soc)*pp.fc.P_rated;
if pp.sc.use
    e = (s.v_sc/pp.sc.Vmax)^2;
    Psc = (P_dem - es.P_lf) - k_e*(0.5 - e)*pp.fc.P_rated;
else
    Psc = 0;
end
es.prev_sc = Psc;
end
