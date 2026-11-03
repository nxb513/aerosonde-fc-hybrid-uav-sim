function s = ps_init(pp, soc0, esc0, P0)
%PS_INIT Trang thai ban dau cua he nguon.
%   s = PS_INIT(pp, soc0, esc0, P0): soc0 pin, esc0 = (v_sc/Vmax)^2, P0 tai dau (W)
%   (FC khoi dong o dong xac lap cap P0 + tai nen, khong qua dinh muc; khau tre kich hoat
%   o xac lap). s.lim dem thoi gian (s) cham tung gioi han cung (ps_step).
if nargin < 2, soc0 = 0.8; end
if nargin < 3, esc0 = 0.5; end
if nargin < 4, P0 = 150; end
fc = pp.fc;
s.it    = (1 - soc0)*pp.bat.cell.Q;
s.istar = 0;
s.soc0  = soc0;
s.v_sc  = sqrt(esc0)*pp.sc.Vmax;
s.esc0  = esc0;
Pst = min((P0 + pp.aux.P)/pp.dc.eta, fc.P_rated);
I = fc.Imin;
for k = 1:30                          % diem lam viec xac lap
    [~, va] = fc_voltage(I, 0, fc);
    V = fc_voltage(I, va, fc);
    I = min(max(Pst/max(V, 1), fc.Imin), fc.I_rated);
end
[~, s.v_act] = fc_voltage(I, 0, fc);
s.I_fc = I;
s.P_stack_prev = fc_voltage(I, s.v_act, fc)*I;
s.m_H2 = 0;
s.E_fc_bus = 0; s.E_sc_bus = 0; s.E_bat = 0; s.E_load = 0; s.E_short = 0; s.E_dump = 0;
s.fc_dP = 0; s.fc_idle_t = 0; s.fc_high_t = 0;
s.bat_Ah = 0; s.bat_dmg = 0;
s.lim = struct('fc_ramp', 0, 'fc_pwr', 0, 'sc', 0, 'bat_dis', 0, 'bat_chg', 0, ...
               'bat_full', 0, 'dump', 0);
s.ems = struct();
s.V_bus = bat_voltage(s.it, 0, pp.bat);
end
