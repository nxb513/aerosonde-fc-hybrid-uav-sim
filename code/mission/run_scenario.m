function res = run_scenario(sc)
%RUN_SCENARIO Mot doan nhiem vu: Aerosonde-FC 6-DOF + autopilot + gio + he nguon trong vong.
%   res = RUN_SCENARIO(sc), sc xem scenario_default. Tra ve chi so vo huong va chuoi
%   P_load (de danh gia lai EMS ngoai vong).
%   Nhiem vu: bay thang o trim (Va cua ung vien) -> tu t_start doi huong dchi va leo
%   dh voi toc do leo toi da Va*sin(gamma_max); bank bi chan o phi_max.
%   Dien ap bus tu he nguon dua vao dong co (ac_propulsion) -> ghep hai chieu.
%   Nhan co cau chap hanh: muc dung lenh u = max_i |cmd_i|/max_i (elevator, aileron,
%   rudder); u > 1 = lenh vuot gioi han lai.
sc = scenario_default(sc);
d2r = pi/180;
I = ac_state_index();
res = struct('ok', false, 'msg', '');

%% Cau hinh
p = ac_params(struct('alpha0_deg', sc.alpha0_deg, 'dmax_deg', sc.dmax_deg, ...
    'rate_deg', sc.rate_deg, 'delay', sc.delay));
if sc.aero_seed > 0
    rs = RandStream('mt19937ar', 'Seed', 1e5 + sc.aero_seed);
    fn = {'CL0','Cm0','CLa','Cma','CLq','Cmq','CLde','CDde','Cmde','CDp','CYb','Clb','Cnb', ...
          'Clp','Cnp','Clr','Cnr','CYda','Clda','Cnda','CYdr','Cldr','Cndr'};
    for k = 1:numel(fn)
        p.aero.(fn{k}) = p.aero.(fn{k})*(1 + p.aero_rel_std*randn(rs));
    end
end
pp = power_params(sc);
[comps, cinfo] = hybrid_config(pp, sc.dx_cg, sc.m_remove, p.mass.m0);
p = ac_apply_config(p, comps);
res.mass = p.mass.m; res.x_cg = p.mass.r_cg(1); res.Jy = p.mass.J(2, 2);
res.m_added = cinfo.m_added;
res.SM = -p.aero.Cma/p.aero.CLa + p.mass.r_cg(1)/p.geom.c;

ps = ps_init(pp, sc.soc0, sc.esc0, 400);
tr = ac_trim(p, sc.Va, 0, struct('Ub', ps.V_bus));
res.trim_alpha = tr.alpha/d2r; res.trim_de = tr.de/d2r; res.trim_dt = tr.dt;
res.trim_margin = tr.margin/d2r; res.trim_Pel = tr.Pel;
res.trim_feasible = tr.feasible;
if ~tr.converged || ~tr.ok_thr
    res.msg = 'trim_failed'; return
end
ap = autopilot_design(p, tr);
ps = ps_init(pp, sc.soc0, sc.esc0, tr.Pel);
ems = str2func(['ems_' sc.ems]);
w = dryden_init(sc.W_h, sc.wind_dir, sc.h0, sc.turb, sc.Va, sc.dt, sc.wind_seed);

%% Mo phong
dt = sc.dt; N = round(sc.T/dt);
x = tr.x0; x(I.pos) = [0; 0; -sc.h0];
nS = round(p.act.delay/dt); nT = round(p.act.thr_delay/dt);
bufS = repmat(tr.cmd(1:3), 1, nS + 1); bufT = repmat(tr.cmd(4), 1, nT + 1);
amax = p.act.max; arate = p.act.rate;
chi0 = 0; chi_t = chi0 + sc.dchi*d2r; h_t = sc.h0 + sc.dh;
climb = sc.Va*sin(sc.gamma_max*d2r);
ast = [];
every = round(1/(sc.trace_hz*dt)); nlog = floor((N - 1)/every) + 1; kl = 0;
L = struct('t', zeros(nlog,1), 'P', zeros(nlog,1), 'V', zeros(nlog,1), 'Ib', zeros(nlog,1), ...
           'Pfc', zeros(nlog,1), 'Psc', zeros(nlog,1), 'h', zeros(nlog,1), 'chi', zeros(nlog,1), ...
           'alpha', zeros(nlog,1), 'Va', zeros(nlog,1), 'dl', zeros(nlog,3));
usage = zeros(N, 1); u_surf = zeros(1, 3); m_rate = 0;
sat_pos_t = 0; sat_rate_t = 0; thr_sat_t = 0; phi_sat_t = 0;
a_hist = zeros(N, 1); a_max = -inf;
E_mech = 0; path = 0; P_peak = 0; Ub = ps.V_bus; in_tol_since = NaN;
for k = 1:N
    t = (k - 1)*dt;
    q = x(I.q); R = quat_utils('dcm', q); eul = quat_utils('to_euler', q);
    vned = R*x(I.vel);
    w = dryden_step(w);
    wv = w.mean_ned; gb = w.gust_b;
    Va = norm(x(I.vel) - R.'*wv - gb);
    meas = struct('phi', eul(1), 'theta', eul(2), 'p', x(I.om(1)), 'q', x(I.om(2)), ...
                  'r', x(I.om(3)), 'chi', atan2(vned(2), vned(1)), 'h', -x(I.pos(3)), 'Va', Va);
    if t < sc.t_start
        ref = struct('chi_c', chi0, 'h_c', sc.h0, 'Va_c', sc.Va);
    else
        ref = struct('chi_c', chi_t, 'h_c', sc.h0 + min(sc.dh, climb*(t - sc.t_start)), 'Va_c', sc.Va);
    end
    [c, ast, ainfo] = autopilot_step(ap, ast, meas, ref, sc.phi_max*d2r, dt);
    bufS = [bufS(:, 2:end), c(1:3)]; bufT = [bufT(2:end), c(4)];
    ca = [bufS(:, 1); bufT(1)];

    f = @(xx) ac_derivatives(xx, ca, Ub, wv, gb, p);
    [k1, aux] = f(x);
    k2 = f(x + 0.5*dt*k1); k3 = f(x + 0.5*dt*k2); k4 = f(x + dt*k3);

    Pel = aux.prop.Pel;
    [ps, po] = ps_step(pp, ps, Pel, dt, ems);
    Ub = po.V_bus;

    us = abs(c(1:3))./amax;
    usage(k) = max(us); u_surf = max(u_surf, us.');
    m_rate = max(m_rate, max(abs(x(I.ddlt))./arate));
    if usage(k) > 1, sat_pos_t = sat_pos_t + dt; end
    if any(abs(x(I.ddlt)) >= arate*(1 - 1e-6)), sat_rate_t = sat_rate_t + dt; end
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
        L.alpha(kl) = aux.alpha; L.Va(kl) = Va; L.dl(kl, :) = x(I.dlt).';
    end

    x = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
    x(I.q) = x(I.q)/norm(x(I.q));
    x(I.dlt) = min(max(x(I.dlt), -amax), amax);
    x(I.ddlt) = min(max(x(I.ddlt), -arate), arate);
    if any(~isfinite(x)) || -x(I.pos(3)) < 5
        res.msg = 'diverged'; break
    end
end
fn = fieldnames(L);
for j = 1:numel(fn), L.(fn{j}) = L.(fn{j})(1:kl, :); end

%% Ket qua
res.ok = ~strcmp(res.msg, 'diverged');
res.m_cmd = max(usage(1:k));
res.u_e = u_surf(1); res.u_a = u_surf(2); res.u_r = u_surf(3);
res.m_rate = m_rate; res.u_p99 = pct(usage(1:k), 99);
res.sat_pos_t = sat_pos_t; res.sat_rate_t = sat_rate_t;
res.thr_sat_t = thr_sat_t; res.phi_sat_t = phi_sat_t;
res.alpha_max = a_max/d2r; res.alpha_p99 = pct(a_hist(1:k), 99)/d2r;
res.alpha_lim = p.alpha_lim/d2r;
res.t_done = in_tol_since - sc.t_start;
res.completed = ~isnan(in_tol_since);
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
% Phan vi (khong can Statistics Toolbox)
x = sort(x(:)); n = numel(x);
r = 1 + (n - 1)*q/100; lo = floor(r); hi = min(lo + 1, n);
v = x(lo) + (r - lo)*(x(hi) - x(lo));
end
