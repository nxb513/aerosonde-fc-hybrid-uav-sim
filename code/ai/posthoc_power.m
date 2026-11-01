function r = posthoc_power(P, dt, cfg, ems, soc0)
%POSTHOC_POWER Danh gia lai he nguon tren chuoi tai P_load(t) da ghi tu E2.
%   r = POSTHOC_POWER(P, dt, cfg, ems, soc0)
%   GT: tai cong suat dong co phu thuoc yeu vao dien ap bus khi autopilot giu
%   van toc khi, nen co the doi EMS / kien truc / sieu tu ma khong bay lai.
%   Chi so:
%     H2 tuong duong duy tri dien tich: m_H2 + (dE_pin + dE_sieutu)/(eta_FC * LHV)
%       (quy doi kieu ECMS [R30]); eta_FC = E_FC_bus/(m_H2 * LHV) cua chinh lan chay.
%     Suy giam FC (uV/cell) = k*(0.4185*dP/(2 P_dinhmuc) + 8.662*t_idle + 10.00*t_cao)
%       [R16][R17]; bo qua start-stop trong mot doan.
%     Mat dung luong pin (%) theo [R18] (bat_damage_rate).
if nargin < 5, soc0 = 0.8; end
P = double(P);
pp = power_params(cfg);
s = ps_init(pp, soc0, 0.5, P(1));
Vmin = inf; Ibmax = 0;
for k = 1:numel(P)
    [s, o] = ps_step(pp, s, P(k), dt, ems);
    Vmin = min(Vmin, o.V_bus); Ibmax = max(Ibmax, o.I_bat);
end
LHV = 120e6;
etaF = s.E_fc_bus/max(s.m_H2*LHV, eps);
dE = (s.soc0 - o.soc)*pp.bat.E_Wh*3600 + 0.5*pp.sc.C*((sqrt(s.esc0)*pp.sc.Vmax)^2 - s.v_sc^2)*pp.sc.use;
r.m_H2_eq = s.m_H2 + dE/(max(etaF, 0.2)*LHV);
fd = pp.fc.deg;
r.fc_uV = fd.k*(fd.load_uV*s.fc_dP/(2*pp.fc.P_rated) + fd.idle_uV_h*s.fc_idle_t/3600 + fd.high_uV_h*s.fc_high_t/3600);
r.bat_loss_pct = s.bat_dmg;
r.bat_limit_t = s.bat_limit; r.sc_limit_t = s.sc_limit;
r.V_min = Vmin; r.I_bat_max = Ibmax; r.soc_end = o.soc;
end
