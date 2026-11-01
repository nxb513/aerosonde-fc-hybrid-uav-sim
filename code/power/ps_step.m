function [s, o] = ps_step(pp, s, P_load, dt, ems)
%PS_STEP Mot buoc he nguon (ban chu dong hoac thu dong).
%   [s, o] = PS_STEP(pp, s, P_load, dt, ems)
%     P_load  cong suat dien dong co (W) lay tu mo hinh he day; cong them tai nen
%     ems     @(pp, s, P_dem, dt) -> [Pfc_ref, Psc_ref, ems_state] (cong suat phia bus)
%   Ban chu dong [R3]: pin noi thang bus; FC, sieu tu qua DC-DC hieu suat eta.
%   Thu dong [R5][R24]: dien ap bus giai tu can bang dong (xem ps_step_passive).

if strcmp(pp.cfg.arch, 'passive')
    [s, o] = ps_step_passive(pp, s, P_load, dt);
    return
end
fc = pp.fc; bat = pp.bat; sc = pp.sc; eta = pp.dc.eta;
P_dem = P_load + pp.aux.P;
[Pfc_ref, Psc_ref, s.ems] = ems(pp, s, P_dem, dt);

%% FC: lenh dong, gioi han toc do tang dong [R25], khau tre kich hoat [R19]
V_now = fc_voltage(s.I_fc, s.v_act, fc);
I_tgt = min(max(Pfc_ref/eta/max(V_now, 1), fc.Imin), fc.Imax);
dI = I_tgt - s.I_fc;
dI = min(dI, fc.ramp*dt);               % chi gioi han chieu tang (GT); giam tu do
s.I_fc = s.I_fc + dI;
[~, va_ss] = fc_voltage(s.I_fc, 0, fc);
s.v_act = va_ss + (s.v_act - va_ss)*exp(-dt/fc.tau);
V_fc = fc_voltage(s.I_fc, s.v_act, fc);
P_stack = V_fc*s.I_fc;
P_fc_bus = eta*P_stack;

%% Sieu tu (C-ESR, xap xi cua [R20])
P_sc_bus = 0; I_sc = 0;
if sc.use && Psc_ref ~= 0
    vmin = sqrt(sc.e_min)*sc.Vmax;
    if (Psc_ref > 0 && s.v_sc <= vmin) || (Psc_ref < 0 && s.v_sc >= sc.Vmax)
        Psc_ref = 0; s.sc_limit = s.sc_limit + dt;
    end
    if Psc_ref > 0, Pt = Psc_ref/eta; else, Pt = Psc_ref*eta; end
    disc = s.v_sc^2 - 4*sc.ESR*Pt;
    if disc < 0, disc = 0; s.sc_limit = s.sc_limit + dt; end
    I_sc = (s.v_sc - sqrt(disc))/(2*sc.ESR);
    if abs(I_sc) > sc.Imax
        I_sc = sign(I_sc)*sc.Imax; s.sc_limit = s.sc_limit + dt;
    end
    Pt = (s.v_sc - sc.ESR*I_sc)*I_sc;
    if Pt > 0, P_sc_bus = Pt*eta; else, P_sc_bus = Pt/eta; end
    s.v_sc = s.v_sc - I_sc*dt/sc.C;
end

%% Pin noi bus: giai P_bat = (E - Rp I) I
P_bat = P_dem - P_fc_bus - P_sc_bus;
[E, Rp] = bat_voltage(s.it, s.istar, bat);
disc = E^2 - 4*Rp*P_bat;
if disc < 0
    disc = 0; s.bat_limit = s.bat_limit + dt;     % vuot cong suat cuc dai cua pin
end
I_b = (E - sqrt(disc))/(2*Rp);
if abs(I_b) > bat.Imax_C*bat.Q
    s.bat_limit = s.bat_limit + dt;
end
V_bus = E - Rp*I_b;
Ic = I_b/bat.Np;
s.it    = s.it + Ic*dt/3600;
s.istar = s.istar + dt/bat.tau_f*(Ic - s.istar);

%% H2 [R24], tich luy suy giam [R16][R17][R18]
etaF = fc.eta_F_hi; if P_stack < fc.P_eta_switch, etaF = fc.eta_F_lo; end
s.m_H2 = s.m_H2 + fc.Ncell*s.I_fc/(2*pp.F)*pp.M_H2/etaF*dt;
s.fc_dP = s.fc_dP + abs(P_stack - s.P_stack_prev);
s.P_stack_prev = P_stack;
if P_stack < fc.deg.idle_frac*fc.P_rated, s.fc_idle_t = s.fc_idle_t + dt; end
if P_stack > fc.deg.high_frac*fc.P_rated, s.fc_high_t = s.fc_high_t + dt; end
dAh = abs(Ic)*dt/3600;
s.bat_Ah = s.bat_Ah + dAh;
s.bat_dmg = s.bat_dmg + bat_damage_rate(abs(I_b)/bat.Q, bat)*dAh;
s.E_fc_bus = s.E_fc_bus + P_fc_bus*dt;
s.E_sc_bus = s.E_sc_bus + P_sc_bus*dt;
s.E_bat = s.E_bat + V_bus*I_b*dt;
s.E_load = s.E_load + P_dem*dt;
s.V_bus = V_bus;

o.V_bus = V_bus; o.I_bat = I_b; o.P_bat = V_bus*I_b;
o.P_fc = P_fc_bus; o.P_sc = P_sc_bus; o.P_dem = P_dem;
o.I_fc = s.I_fc; o.V_fc = V_fc; o.I_sc = I_sc;
o.soc = 1 - s.it/bat.cell.Q; o.esc = (s.v_sc/sc.Vmax)^2;
end
