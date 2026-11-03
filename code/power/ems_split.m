function [Pfc, Psc, es] = ems_split(pp, s, P_dem, dt, f_c, es)
%EMS_SPLIT Tach cong suat theo tan so [R3] voi tan so cat f_c (Hz) cho buoc nay.
%   FC: LPF(P_dem, f_c) + k_soc (SoC_ref - SoC) P_fc; sieu tu: phan con lai cua P_dem
%   tru di k_e (0.5 - e) P_fc (e = nang luong sieu tu tuong doi). Khong co sieu tu thi
%   Psc = 0 va pin bu phan tan so cao. k_soc, k_e: pp.ems (GT).
a = 1 - exp(-2*pi*f_c*dt);
es.P_lf = es.P_lf + a*(P_dem - es.P_lf);
soc = 1 - s.it/pp.bat.cell.Q;
Pfc = es.P_lf + pp.ems.k_soc*(s.soc0 - soc)*pp.fc.P_rated;
if pp.sc.use
    e = (s.v_sc/pp.sc.Vmax)^2;
    Psc = (P_dem - es.P_lf) - pp.ems.k_e*(0.5 - e)*pp.fc.P_rated;
else
    Psc = 0;
end
end
