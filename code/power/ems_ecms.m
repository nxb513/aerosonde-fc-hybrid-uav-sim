function [Pfc, Psc, es] = ems_ecms(pp, s, P_dem, dt)
%EMS_ECMS Equivalent Consumption Minimization Strategy [R30] cho cap FC/pin.
%   Moi buoc chon P_fc tren duong cong xac lap cua FC (toi cong suat dinh muc) de cuc
%   tieu J = mdot_H2(P_fc) + s_eq * P_bat / LHV, voi s_eq = s0 (1 + k (SoC_ref - SoC)),
%   s0 = 2.5, k = 3 (GT, cung gia tri voi he so tuong duong trong ham chi phi planner).
%   Sieu tu nhan phan tan so cao nhu ems_filter (cung f_c), de so sanh chi khac cach chia
%   FC/pin. LHV H2 = 120 MJ/kg.
es = s.ems;
if ~isfield(es, 'curve')
    fc = pp.fc;
    I = linspace(fc.Imin, fc.I_rated, 41);
    P = zeros(size(I)); md = P;
    for k = 1:numel(I)
        [~, va] = fc_voltage(I(k), 0, fc);
        P(k) = fc_voltage(I(k), va, fc)*I(k);
        etaF = fc.eta_F_hi; if P(k) < fc.P_eta_switch, etaF = fc.eta_F_lo; end
        md(k) = fc.Ncell*I(k)/(2*pp.F)*pp.M_H2/etaF;
    end
    es.curve = struct('Pbus', pp.dc.eta*P, 'mdot', md);
    es.P_lf = P_dem;
end
LHV = 120e6;
soc = 1 - s.it/pp.bat.cell.Q;
seq = 2.5*(1 + 3*(s.soc0 - soc));
if pp.sc.use
    a = 1 - exp(-2*pi*pp.ems.f_c*dt);
    es.P_lf = es.P_lf + a*(P_dem - es.P_lf);
    e = (s.v_sc/pp.sc.Vmax)^2;
    Psc = (P_dem - es.P_lf) - pp.ems.k_e*(0.5 - e)*pp.fc.P_rated;
    Pres = es.P_lf;
else
    Psc = 0; Pres = P_dem;
end
J = es.curve.mdot + seq*(Pres - es.curve.Pbus)/LHV;
[~, k] = min(J);
Pfc = es.curve.Pbus(k);
end
