function [s, o] = ps_step(pp, s, P_load, dt, ems)
%PS_STEP Mot buoc he nguon ban chu dong [R3]: pin noi bus; FC, sieu tu qua DC-DC.
%   [s, o] = PS_STEP(pp, s, P_load, dt, ems)
%     P_load  cong suat dien dong co (W) tu mo hinh he day; cong them tai nen
%     ems     @(pp, s, P_dem, dt) -> [Pfc_ref, Psc_ref, ems_state] (cong suat phia bus)
%   Moi gioi han cung duoc ap o moi buoc (khong bao gio vuot):
%     FC      dong trong [Imin, Imax]; tang dong <= ramp*dt [R25] (giam tu do, GT);
%             cong suat stack <= P_rated.
%     Sieu tu nang luong 25-100 % [R3], dong <= Imax; dong bi kep de khong vuot bien
%             trong buoc.
%     Pin     dong xa <= I_dis_max (10C); dong sac <= I_chg_max (1C); het sac khi SoC = 1.
%   Pin la nguon cuoi cung:
%     - can xa qua gioi han -> pin giu o gioi han, tai thieu o.P_short (dong co nhan it
%       hon o buoc sau qua o.P_motor_cap, xem run_flight);
%     - du cong suat qua muc pin duoc nap -> giam FC (giam dong tu do); neu FC da o Imin
%       ma van du thi phan du dot tren dien tro xa (GT), ghi o.P_dump.
%   Can bang: P_fc + P_sc + P_bat = P_dem - P_short + P_dump.
fc = pp.fc; bat = pp.bat; sc = pp.sc; eta = pp.dc.eta;
P_dem = P_load + pp.aux.P;
[Pfc_ref, Psc_ref, s.ems] = ems(pp, s, P_dem, dt);

%% FC: lenh dong, gioi han tang dong [R25], tre kich hoat [R19], cong suat <= dinh muc
I0 = s.I_fc; va0 = s.v_act;
V_now = fc_voltage(I0, va0, fc);
I_tgt = min(max(Pfc_ref/eta/max(V_now, 1), fc.Imin), fc.Imax);
I_fc = I0 + min(I_tgt - I0, fc.ramp*dt);
if I_tgt - I0 > fc.ramp*dt, s.lim.fc_ramp = s.lim.fc_ramp + dt; end
if fc_power(I_fc, va0, fc, dt) > fc.P_rated
    I_fc = fc_solve(@(I) fc_power(I, va0, fc, dt) - fc.P_rated, fc.Imin, I_fc);
    s.lim.fc_pwr = s.lim.fc_pwr + dt;
end
[P_stack, V_fc, va1] = fc_power(I_fc, va0, fc, dt);
P_fc_bus = eta*P_stack;

%% Sieu tu (C-ESR, xap xi cua [R20])
P_sc_bus = 0; I_sc = 0;
if sc.use && Psc_ref ~= 0
    vmin = sqrt(sc.e_min)*sc.Vmax;
    if (Psc_ref > 0 && s.v_sc <= vmin) || (Psc_ref < 0 && s.v_sc >= sc.Vmax)
        Psc_ref = 0; s.lim.sc = s.lim.sc + dt;
    end
    if Psc_ref > 0, Pt = Psc_ref/eta; else, Pt = Psc_ref*eta; end
    disc = s.v_sc^2 - 4*sc.ESR*Pt;
    if disc < 0, disc = 0; s.lim.sc = s.lim.sc + dt; end
    I_sc = (s.v_sc - sqrt(disc))/(2*sc.ESR);
    I_hi = min(sc.Imax, max(s.v_sc - vmin, 0)*sc.C/dt);       % khong xa qua 25 %
    I_lo = -min(sc.Imax, max(sc.Vmax - s.v_sc, 0)*sc.C/dt);   % khong nap qua 100 %
    if I_sc > I_hi || I_sc < I_lo
        I_sc = min(max(I_sc, I_lo), I_hi); s.lim.sc = s.lim.sc + dt;
    end
    Pt = (s.v_sc - sc.ESR*I_sc)*I_sc;
    if Pt > 0, P_sc_bus = Pt*eta; else, P_sc_bus = Pt/eta; end
end

%% Pin noi bus: P_bat = (E - Rp I) I
[E, Rp] = bat_voltage(s.it, s.istar, bat);
P_bat = P_dem - P_fc_bus - P_sc_bus;
I_b = bat_current(P_bat, E, Rp);
P_short = 0; P_dump = 0;
% Nap: <= 1C va khong vuot SoC = 1 trong buoc nay
I_chg = min(bat.I_chg_max, max(s.it, 0)*3600/dt*bat.Np);
if I_b < -I_chg
    P_allow = -(E + Rp*I_chg)*I_chg;                 % P_bat khi I = -I_chg
    target = P_fc_bus - (P_allow - P_bat);           % giam FC dung bang phan du
    if target < eta*fc_power(fc.Imin, va0, fc, dt)
        I_fc = fc.Imin;
    else
        I_fc = fc_solve(@(I) eta*fc_power(I, va0, fc, dt) - target, fc.Imin, I_fc);
    end
    [P_stack, V_fc, va1] = fc_power(I_fc, va0, fc, dt);
    P_fc_bus = eta*P_stack;
    P_bat = P_dem - P_fc_bus - P_sc_bus;
    if P_bat < P_allow
        P_dump = P_allow - P_bat; P_bat = P_allow;
        s.lim.dump = s.lim.dump + dt;
    end
    I_b = max(bat_current(P_bat, E, Rp), -I_chg);
    if I_chg < bat.I_chg_max, s.lim.bat_full = s.lim.bat_full + dt;
    else, s.lim.bat_chg = s.lim.bat_chg + dt; end
end
% Xa: <= 10C; phan thieu khong cap duoc cho tai
if I_b > bat.I_dis_max
    I_b = bat.I_dis_max;
    P_lim = (E - Rp*I_b)*I_b;
    P_short = P_bat - P_lim; P_bat = P_lim;
    s.lim.bat_dis = s.lim.bat_dis + dt;
end
V_bus = E - Rp*I_b;
Ic = I_b/bat.Np;
s.it    = s.it + Ic*dt/3600;
s.istar = s.istar + dt/bat.tau_f*(Ic - s.istar);

%% Cap nhat FC, sieu tu
s.I_fc = I_fc; s.v_act = va1;
if sc.use, s.v_sc = s.v_sc - I_sc*dt/sc.C; end

%% H2 [R24], tich luy suy giam [R16][R17][R18]
etaF = fc.eta_F_hi; if P_stack < fc.P_eta_switch, etaF = fc.eta_F_lo; end
s.m_H2 = s.m_H2 + fc.Ncell*I_fc/(2*pp.F)*pp.M_H2/etaF*dt;
s.fc_dP = s.fc_dP + abs(P_stack - s.P_stack_prev);
s.P_stack_prev = P_stack;
if P_stack <= fc.deg.idle_frac*fc.P_rated, s.fc_idle_t = s.fc_idle_t + dt; end
if P_stack >= fc.deg.high_frac*fc.P_rated, s.fc_high_t = s.fc_high_t + dt; end
dAh = abs(Ic)*dt/3600;
s.bat_Ah = s.bat_Ah + dAh;
s.bat_dmg = s.bat_dmg + bat_damage_rate(abs(I_b)/bat.Q, bat)*dAh;
s.E_fc_bus = s.E_fc_bus + P_fc_bus*dt;
s.E_sc_bus = s.E_sc_bus + P_sc_bus*dt;
s.E_bat = s.E_bat + V_bus*I_b*dt;
s.E_load = s.E_load + (P_dem - P_short)*dt;
s.E_short = s.E_short + P_short*dt;
s.E_dump = s.E_dump + P_dump*dt;
s.V_bus = V_bus;

o.V_bus = V_bus; o.I_bat = I_b; o.P_bat = P_bat;
o.P_fc = P_fc_bus; o.P_sc = P_sc_bus; o.P_dem = P_dem;
o.P_short = P_short; o.P_dump = P_dump;
o.I_fc = I_fc; o.V_fc = V_fc; o.P_stack = P_stack; o.I_sc = I_sc;
o.soc = 1 - s.it/bat.cell.Q; o.esc = (s.v_sc/sc.Vmax)^2;
o.E = E; o.Rp = Rp;
% Cong suat toi da dong co nhan duoc o buoc sau: FC + sieu tu hien tai + pin o gioi han xa
o.P_motor_cap = P_fc_bus + P_sc_bus + (E - Rp*bat.I_dis_max)*bat.I_dis_max - pp.aux.P;
end

function I = bat_current(P, E, Rp)
% Dong pin cho cong suat P = (E - Rp I) I; vuot cong suat cuc dai thi lay dinh E/(2 Rp)
disc = E^2 - 4*Rp*P;
if disc < 0, I = E/(2*Rp); else, I = (E - sqrt(disc))/(2*Rp); end
end

function [P, V, va1] = fc_power(I, va0, fc, dt)
% Cong suat stack sau buoc dt neu dong la I (khau tre kich hoat cap nhat trong buoc)
[~, va_ss] = fc_voltage(I, 0, fc);
va1 = va_ss + (va0 - va_ss)*exp(-dt/fc.tau);
V = fc_voltage(I, va1, fc);
P = V*I;
end

function I = fc_solve(g, lo, hi)
% Nghiem cua g(I) = 0 tren [lo, hi], g tang theo I; tra ve diem g <= 0 (khong vuot)
for k = 1:50
    mid = 0.5*(lo + hi);
    if g(mid) > 0, hi = mid; else, lo = mid; end
end
I = lo;
end
