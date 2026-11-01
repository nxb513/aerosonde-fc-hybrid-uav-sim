function [s, o] = ps_step_passive(pp, s, P_load, dt)
%PS_STEP_PASSIVE Kien truc thu dong: FC (co diode), pin, sieu tu song song tren bus.
%   Chia dong theo dien ap nhu [R5][R24]: dong FC khong am (diode), khong co
%   lenh dieu khien. Dien ap bus V giai tu: I_fc(V) + I_bat(V) + I_sc(V) = P_dem/V.
fc = pp.fc; bat = pp.bat; sc = pp.sc;
P_dem = P_load + pp.aux.P;
[E, Rp] = bat_voltage(s.it, s.istar, bat);

Ifc_of_V = @(V) fc_current_at(V, s.v_act, fc);
Isc_of_V = @(V) sc.use*(s.v_sc - V)/sc.ESR;
g = @(V) Ifc_of_V(V) + (E - V)/Rp + Isc_of_V(V) - P_dem/V;
lo = 0.5*E; hi = max([E, s.v_sc, fc_voltage(fc.Imin, s.v_act, fc)]) + 1;
if g(lo)*g(hi) > 0
    V = E - Rp*P_dem/E; s.bat_limit = s.bat_limit + dt;
else
    V = fzero(g, [lo hi]);
end
s.I_fc = Ifc_of_V(V);
I_b = (E - V)/Rp;
I_sc = Isc_of_V(V);

[~, va_ss] = fc_voltage(max(s.I_fc, fc.Imin), 0, fc);
s.v_act = va_ss + (s.v_act - va_ss)*exp(-dt/fc.tau);
if sc.use, s.v_sc = s.v_sc - (I_sc + sc.ESR*0)*dt/sc.C; end
Ic = I_b/bat.Np;
s.it = s.it + Ic*dt/3600;
s.istar = s.istar + dt/bat.tau_f*(Ic - s.istar);
if abs(I_b) > bat.Imax_C*bat.Q, s.bat_limit = s.bat_limit + dt; end

V_fc = V;  P_stack = V_fc*s.I_fc;
etaF = fc.eta_F_hi; if P_stack < fc.P_eta_switch, etaF = fc.eta_F_lo; end
s.m_H2 = s.m_H2 + fc.Ncell*s.I_fc/(2*pp.F)*pp.M_H2/etaF*dt;
s.fc_dP = s.fc_dP + abs(P_stack - s.P_stack_prev);
s.P_stack_prev = P_stack;
if P_stack < fc.deg.idle_frac*fc.P_rated, s.fc_idle_t = s.fc_idle_t + dt; end
if P_stack > fc.deg.high_frac*fc.P_rated, s.fc_high_t = s.fc_high_t + dt; end
dAh = abs(Ic)*dt/3600;
s.bat_Ah = s.bat_Ah + dAh;
s.bat_dmg = s.bat_dmg + bat_damage_rate(abs(I_b)/bat.Q, bat)*dAh;
s.E_fc_bus = s.E_fc_bus + P_stack*dt;
s.E_sc_bus = s.E_sc_bus + V*I_sc*dt;
s.E_bat = s.E_bat + V*I_b*dt;
s.E_load = s.E_load + P_dem*dt;
s.V_bus = V;

o.V_bus = V; o.I_bat = I_b; o.P_bat = V*I_b;
o.P_fc = P_stack; o.P_sc = V*I_sc; o.P_dem = P_dem;
o.I_fc = s.I_fc; o.V_fc = V_fc; o.I_sc = I_sc;
o.soc = 1 - s.it/bat.cell.Q; o.esc = (s.v_sc/sc.Vmax)^2;
end

function I = fc_current_at(V, v_act, fc)
% Dong FC khi dien ap dau cuc = V (diode: I >= 0, I <= Imax)
if fc_voltage(fc.Imin, v_act, fc) <= V
    I = 0; return
end
if fc_voltage(fc.Imax, v_act, fc) >= V
    I = fc.Imax; return
end
I = fzero(@(I) fc_voltage(I, v_act, fc) - V, [fc.Imin fc.Imax]);
end
