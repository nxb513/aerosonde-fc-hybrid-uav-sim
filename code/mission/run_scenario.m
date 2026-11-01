function res = run_scenario(sc)
%RUN_SCENARIO Mo phong mot doan nhiem vu: X8 6-DOF + autopilot + gio + he nguon trong vong.
%   res = RUN_SCENARIO(sc), sc xem scenario_default. Tra ve chi so vo huong va
%   chuoi P_load (de danh gia lai EMS ngoai vong).
%   Nhiem vu: bay thang o trim (Va cua ung vien) -> tu t_start doi huong dchi va
%   leo dh voi toc do leo toi da Va*sin(gamma_max); bank bi chan o phi_max.
%   Dien ap bus tu he nguon dua vao he day [R2, Eq. 28] -> ghep hai chieu.

sc = scenario_default(sc);
d2r = pi/180;
I = x8_state_index();
res = struct('ok', false, 'msg', '');

%% Cau hinh
p = x8_params();
if sc.aero_seed > 0
    rs = RandStream('mt19937ar', 'Seed', 1e5 + sc.aero_seed);
    fn = fieldnames(p.aero_std);
    for k = 1:numel(fn)
        p.aero.(fn{k}) = p.aero.(fn{k}) + p.aero_std.(fn{k})*randn(rs);
    end
end
pp = power_params(sc);
[comps, cinfo] = hybrid_config(pp, sc.dx_cg, sc.m_remove);
p = x8_apply_config(p, comps);
res.mass = p.mass.m; res.x_cg = p.mass.r_cg(1); res.Jy = p.mass.J(2, 2);
res.m_added = cinfo.m_added;

ps = ps_init(pp, sc.soc0, sc.esc0, 150);
tr = x8_trim(p, sc.Va, 0, struct('Ub', ps.V_bus));
res.trim_alpha = tr.alpha/d2r; res.trim_de = tr.de/d2r; res.trim_dt = tr.dt;
res.trim_margin = tr.elevon_margin/d2r; res.trim_Pel = tr.Pel;
res.trim_feasible = tr.feasible;
if ~tr.converged || ~tr.ok_throttle
    res.msg = 'trim_failed'; return
end
ap = autopilot_design(p, tr);
ps = ps_init(pp, sc.soc0, sc.esc0, tr.Pel);
ems = str2func(['ems_' sc.ems]);

%% Gio
w = dryden_init(sc.W_h, sc.wind_dir, sc.h0, sc.turb, sc.Va, sc.dt, sc.wind_seed);

%% Mo phong
dt = sc.dt; N = round(sc.T/dt);
x = tr.x0; x(I.pos) = [0; 0; -sc.h0];
nE = round(p.act.elevon.delay/dt); nT = round(p.act.throttle.delay/dt);
bufE = repmat(tr.cmd(1:2), 1, nE + 1); bufT = repmat(tr.cmd(3), 1, nT + 1);
emax = p.act.elevon.max; erate = p.act.elevon.rate;
chi0 = 0; chi_t = chi0 + sc.dchi*d2r; h_t = sc.h0 + sc.dh;
climb = sc.Va*sin(sc.gamma_max*d2r);
ast = [];
nlog = floor(N/round(1/(sc.trace_hz*dt))) + 1; kl = 0; every = round(1/(sc.trace_hz*dt));
L = struct('t', zeros(nlog,1), 'P', zeros(nlog,1), 'V', zeros(nlog,1), 'Ib', zeros(nlog,1), ...
           'Pfc', zeros(nlog,1), 'Psc', zeros(nlog,1), 'h', zeros(nlog,1), 'chi', zeros(nlog,1), ...
           'alpha', zeros(nlog,1), 'Va', zeros(nlog,1), 'der', zeros(nlog,1), 'del', zeros(nlog,1));
m_cmd = 0; m_rate = 0; sat_pos_t = 0; sat_rate_t = 0; thr_sat_t = 0; phi_sat_t = 0;
a_max = -inf; a_hist = zeros(N, 1); usage = zeros(N, 1);
E_mech = 0; path = 0; P_peak = 0; Ub = ps.V_bus;
done_t = NaN; in_tol_since = NaN;
for k = 1:N
    t = (k - 1)*dt;
    q = x(I.q); R = quat_utils('dcm', q); eul = quat_utils('to_euler', q);
    vned = R*x(I.vel);
    w = dryden_step(w);
    wv = w.mean_ned; gb = w.gust_b;
    vr = x(I.vel) - R.'*wv - gb; Va = norm(vr);
    meas = struct('phi', eul(1), 'theta', eul(2), 'p', x(I.om(1)), 'q', x(I.om(2)), ...
                  'chi', atan2(vned(2), vned(1)), 'h', -x(I.pos(3)), 'Va', Va);
    if t < sc.t_start
        ref = struct('chi_c', chi0, 'h_c', sc.h0, 'Va_c', sc.Va);
    else
        ref = struct('chi_c', chi_t, 'h_c', sc.h0 + min(sc.dh, climb*(t - sc.t_start)), 'Va_c', sc.Va);
    end
    [c, ast, ainfo] = autopilot_step(ap, ast, meas, ref, sc.phi_max*d2r, dt);
    bufE = [bufE(:, 2:end), c(1:2)]; bufT = [bufT(2:end), c(3)];
    ca = [bufE(:, 1); bufT(1)];

    f = @(xx) x8_derivatives(xx, ca, Ub, wv, gb, p);
    [k1, aux] = f(x);
    k2 = f(x + 0.5*dt*k1); k3 = f(x + 0.5*dt*k2); k4 = f(x + dt*k3);

    % He nguon (buoc Euler; dong hoc nguon cham hon nhieu so voi dt)
    Pel = aux.prop.Pel;
    [ps, po] = ps_step(pp, ps, Pel, dt, ems);
    Ub = po.V_bus;

    % Chi so
    u_k = max(abs(c(1:2)))/emax;
    usage(k) = u_k; m_cmd = max(m_cmd, u_k);
    m_rate = max(m_rate, max(abs(x(I.ddlt)))/erate);
    if u_k > 1, sat_pos_t = sat_pos_t + dt; end
    if any(abs(x(I.ddlt)) >= erate*(1 - 1e-6)), sat_rate_t = sat_rate_t + dt; end
    if ainfo.sat_dt, thr_sat_t = thr_sat_t + dt; end
    if ainfo.sat_phi, phi_sat_t = phi_sat_t + dt; end
    a_hist(k) = aux.alpha; a_max = max(a_max, aux.alpha);
    E_mech = E_mech + aux.prop.T*Va*dt; path = path + norm(vned(1:2))*dt;
    P_peak = max(P_peak, Pel);
    in_tol = t >= sc.t_start && abs(mod(chi_t - meas.chi + pi, 2*pi) - pi) < 5*d2r && abs(h_t - meas.h) < 3;
    if in_tol
        if isnan(in_tol_since), in_tol_since = t; end
    else
        in_tol_since = NaN;
    end
    if mod(k - 1, every) == 0
        kl = kl + 1;
        L.t(kl) = t; L.P(kl) = Pel; L.V(kl) = po.V_bus; L.Ib(kl) = po.I_bat;
        L.Pfc(kl) = po.P_fc; L.Psc(kl) = po.P_sc; L.h(kl) = meas.h; L.chi(kl) = meas.chi;
        L.alpha(kl) = aux.alpha; L.Va(kl) = Va; L.der(kl) = x(I.dlt(1)); L.del(kl) = x(I.dlt(2));
    end

    x = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
    x(I.q) = x(I.q)/norm(x(I.q));
    x(I.dlt) = min(max(x(I.dlt), -emax), emax);
    x(I.ddlt) = min(max(x(I.ddlt), -erate), erate);
    x(I.Om) = max(x(I.Om), 0);
    if any(~isfinite(x)) || -x(I.pos(3)) < 5
        res.msg = 'diverged'; break
    end
end
done_t = in_tol_since;
fn = fieldnames(L);
for j = 1:numel(fn), L.(fn{j}) = L.(fn{j})(1:kl); end

%% Ket qua
res.ok = ~strcmp(res.msg, 'diverged');
res.m_cmd = m_cmd;                         % >1: lenh elevon vuot +-30 deg
res.m_rate = m_rate;                       % =1: cham gioi han toc do
res.u_p99 = pct(usage(1:k), 99);
res.sat_pos_t = sat_pos_t; res.sat_rate_t = sat_rate_t;
res.thr_sat_t = thr_sat_t; res.phi_sat_t = phi_sat_t;
res.alpha_max = a_max/d2r; res.alpha_p99 = pct(a_hist(1:k), 99)/d2r;
res.t_done = done_t - sc.t_start;          % NaN: khong hoan thanh trong T
res.completed = ~isnan(done_t);
res.h_err_end = h_t + x(I.pos(3)); res.path = path;
res.E_el = ps.E_load; res.E_mech = E_mech; res.P_peak = P_peak;
res.P_mean = ps.E_load/(k*dt);
res.m_H2 = ps.m_H2; res.soc_end = 1 - ps.it/pp.bat.cell.Q; res.esc_end = (ps.v_sc/pp.sc.Vmax)^2;
res.dE_bat = ps.E_bat; res.dE_sc = ps.E_sc_bus;
res.fc_dP = ps.fc_dP; res.fc_idle_t = ps.fc_idle_t; res.fc_high_t = ps.fc_high_t;
res.bat_Ah = ps.bat_Ah; res.bat_dmg = ps.bat_dmg;
res.bat_limit_t = ps.bat_limit; res.sc_limit_t = ps.sc_limit;
res.V_bus_min = min(L.V); res.I_bat_max = max(L.Ib);
res.trace = L;
end

function v = pct(x, q)
% Phan vi (khong can Statistics Toolbox): noi suy tuyen tinh tren mau da sap xep
x = sort(x(:)); n = numel(x);
r = 1 + (n - 1)*q/100; lo = floor(r); hi = min(lo + 1, n);
v = x(lo) + (r - lo)*(x(hi) - x(lo));
end
