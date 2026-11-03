function [Pfc, Psc, es] = ems_filter(pp, s, P_dem, dt)
%EMS_FILTER Chien luoc tach tan so [R3] voi tan so cat co dinh f_c = pp.ems.f_c (EMS
%   chinh cua E3, mac dinh 0.1 Hz, GT).
%   FC nhan thanh phan tan so thap LPF(P_dem, f_c); sieu tu nhan phan tan so cao;
%   pin bu phan con lai (ke ca phan FC khong theo kip do gioi han toc do).
%   Bo sung (GT, he so xem power_params): hieu chinh SoC pin ve SoC dau chuyen bay va
%   nang luong sieu tu ve 50 %.
es = s.ems;
if ~isfield(es, 'P_lf'), es.P_lf = P_dem; end
[Pfc, Psc, es] = ems_split(pp, s, P_dem, dt, pp.ems.f_c, es);
end
