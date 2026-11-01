function [res, F] = run_flight(fl, M, opts)
%RUN_FLIGHT Chuyen bay vong kin E3: bay chuoi nhiem vu den khi het H2 hoac SoC < 30 %.
%   [res, F] = RUN_FLIGHT(fl, M, opts)
%     fl    kich ban (flight_default); M bo mo hinh (e3_train) cho chinh sach 'model'
%     opts  ckpt_file: checkpoint giua chuyen bay ('' = khong luu); ckpt_every: chu ky luu
%           theo thoi gian tinh (s, mac dinh 300). Neu ckpt_file ton tai va dung chuyen
%           bay (id + chinh sach) thi chay tiep tu do.
%   Vong lap: dau moi nhiem vu va moi T_plan giay (tham do: giu ung vien H giay), planner
%   chon (bank toi da, goc leo toi da, van toc, muc bu EMS). Autopilot [R37, ch. 6] bam
%   duong [R37, ch. 10] trong cac gioi han do; 6-DOF + servo + he nguon trong vong.
%   Gain autopilot thiet ke o 3 van toc trim {22, 25, 28} m/s, chon theo van toc lenh;
%   feed-forward trim noi suy theo van toc do (GT).
%   Dung khi: het H2 (fill x dung tich), SoC < soc_stop (du tru ha canh 30 % [R27]),
%   roi (h < 5 m hoac phan ky), hoac T_max. Khoi luong H2 tieu thu (< 0.2 % khoi luong
%   may bay) bo qua trong dong luc hoc (GT).
%   Nhan cua moi cua so quyet dinh (de hoc AFP/mo hinh thay the):
%     m = max(lenh lai / gioi han goc, alpha_p99 / alpha_lim, bao hoa toc do lien tuc / 0.2 s)
%     (bao hoa toc do lai keo dai > 0.2 s tinh la vi pham: GT); toc do tien do (m/s),
%     H2 (kg/s), cong suat pin + sieu tu ra bus (W), suy giam pin [R18] (%/s), bien thien
%     cong suat FC [R16][R17] (W/s), cong suat dinh.
%   Nhiem vu "dat" = hoan thanh truoc han va m cua ca nhiem vu <= 1.
if nargin < 2, M = []; end
if nargin < 3, opts = struct(); end
if ~isfield(opts, 'ckpt_file'), opts.ckpt_file = ''; end
if ~isfield(opts, 'ckpt_every'), opts.ckpt_every = 300; end
fl = flight_default(fl);
F = [];
if ~isempty(opts.ckpt_file) && exist(opts.ckpt_file, 'file')
    S = load(opts.ckpt_file);
    if isfield(S, 'F') && S.F.fl.id == fl.id && strcmp(S.F.fl.policy, fl.policy)
        F = S.F;
        fprintf('Chay tiep chuyen bay %d (%s) tu t = %.0f s\n', fl.id, fl.policy, F.t);
    end
end
if isempty(F)
    [F, res] = flight_init(fl);
    if ~F.ok, return; end
end
t_save = tic; t_wall = tic;
while ~F.stop
    F = flight_segment(F, M);
    if ~isempty(opts.ckpt_file) && ~F.stop && toc(t_save) > opts.ckpt_every
        F.wall = F.wall + toc(t_wall); t_wall = tic;
        save(opts.ckpt_file, 'F', '-v7'); t_save = tic;
    end
end
F.wall = F.wall + toc(t_wall);
res = flight_result(F);
if ~isempty(opts.ckpt_file) && exist(opts.ckpt_file, 'file'), delete(opts.ckpt_file); end
end

%% ------------------------------------------------------------------------------------
function [F, res] = flight_init(fl)
res = struct('ok', false, 'msg', '');
F.ok = false;
pol = policy_def(fl.policy);
p = ac_params(struct('alpha0_deg', fl.alpha0_deg, 'dmax_deg', fl.dmax_deg, ...
    'rate_deg', fl.rate_deg, 'delay', fl.delay));
p = ac_perturb(p, fl.aero_seed);
pp = power_params(fl);
comps = hybrid_config(pp, fl.dx_cg, fl.m_remove, p.mass.m0);
p = ac_apply_config(p, comps);
res.mass = p.mass.m; res.x_cg = p.mass.r_cg(1);
res.SM = -p.aero.Cma/p.aero.CLa + p.mass.r_cg(1)/p.geom.c;
C = e3_candidates();
ps = ps_init(pp, fl.soc0, fl.esc0, 400);
aps = cell(1, 3); ff = zeros(3, 3); trs = cell(1, 3); Va_ok = false(1, 3);
for iv = 1:3
    tr = ac_trim(p, C.Va(iv), 0, struct('Ub', ps.V_bus));
    Va_ok(iv) = tr.converged && tr.ok_thr;
    if Va_ok(iv)
        aps{iv} = autopilot_design(p, tr); ff(iv, :) = [tr.de tr.dt tr.theta]; trs{iv} = tr;
    end
end
if ~Va_ok(2)
    res.msg = 'trim_failed_25'; return
end
% Van toc khong trim duoc (vd. 22 m/s voi cau hinh nang, lai can vuot gioi han): loai
% khoi tap ung vien cua chuyen bay nay; gain/feed-forward lay theo 25 m/s.
for iv = find(~Va_ok), aps{iv} = aps{2}; ff(iv, :) = ff(2, :); end
res.Va22_ok = double(Va_ok(1)); res.Va28_ok = double(Va_ok(3));
F.Va_ok = Va_ok;
F.fl = fl; F.pol = pol; F.p = p; F.pp = pp; F.C = C; F.aps = aps; F.ff = ff;
F.I = ac_state_index();
F.ems = ['ems_' pol.ems];
F.specs = task_specs(fl.seed, fl.W_lvl);
F.H2_avail = fl.fill*pp.fc.m_H2_cap;
F.res0 = res;
F.x = trs{2}.x0; F.x(F.I.pos) = [0; 0; -fl.h0];
F.ps = ps_init(pp, fl.soc0, fl.esc0, trs{2}.Pel);
nS = round(p.act.delay/fl.dt); nT = round(p.act.thr_delay/fl.dt);
F.bufS = repmat(trs{2}.cmd(1:3), 1, nS + 1); F.bufT = repmat(trs{2}.cmd(4), 1, nT + 1);
F.ast = [];
sp = F.specs(1);
F.w = dryden_init(sp.W, sp.dir, fl.h0, sp.turb, 25, fl.dt, fl.wind_seed);
F.Wt = F.w.mean_ned; F.turb_now = sp.turb;
F.Ub = F.ps.V_bus;
F.rs = RandStream('mt19937ar', 'Seed', 9e4 + fl.id);
F.t = 0; F.h_c = fl.h0;
F.cand = [C.traj(C.nominal, :), NaN]; F.iVa = 2;
F.itask = 0; F.need_task = true; F.task = []; F.t_task0 = 0; F.ta = [];
F.gd = []; F.r_prev = 0; F.P_cum = 0;
F.ring = zeros(1, 60); F.iring = 0;
F.rs_run = 0; F.alpha = trs{2}.alpha; F.Pel = trs{2}.Pel;
F.po = struct('P_fc', trs{2}.Pel, 'V_bus', F.ps.V_bus);
F.stop = false; F.reason = '';
F.TK = zeros(0, 17); F.WN = zeros(0, 47, 'single'); F.DL = zeros(0, 10, 'single');
F.viol_time = 0; F.n_dec = 0; F.P_peak = 0; F.wall = 0;
F.ok = true; res.ok = true;
end

%% ------------------------------------------------------------------------------------
function F = flight_segment(F, M)
fl = F.fl; dt = fl.dt; I = F.I; d2r = pi/180;
if F.need_task, F = start_task(F); end

% ---- Quyet dinh
ctx = make_ctx(F);
lam = F.task.lambda; if ~isempty(F.pol.lam), lam = F.pol.lam; end
[it, fcv, info] = planner_decide(F.pol, M, ctx, F.C, lam, F.rs);
cand = [F.C.traj(it, :), fcv];
F.cand = cand; [~, F.iVa] = min(abs(F.C.Va - cand(3)));
if isnan(fcv), F.ps.ems.fc_fix = []; else, F.ps.ems.fc_fix = fcv; end
F.n_dec = F.n_dec + 1;
if strcmp(F.pol.kind, 'explore'), period = fl.H; else, period = fl.T_plan; end
Xw = e3_features(ctx, cand(1:3), max(fcv, 0)*~isnan(fcv));

% ---- Mo phong mot cua so
x = F.x; bufS = F.bufS; bufT = F.bufT; ast = F.ast; ps = F.ps; w = F.w; Ub = F.Ub;
tk = F.task; ta = F.ta; hc = F.h_c; t = F.t; rs_run = F.rs_run; P_cum = F.P_cum;
r_prev = F.r_prev; ring = F.ring; iring = F.iring; viol_time = F.viol_time; Ppk_f = F.P_peak;
p = F.p; pp = F.pp; ems = str2func(F.ems);
amax = p.act.max; arate = p.act.rate; alim = p.alpha_lim;
ap = F.aps{F.iVa}; ffv = F.ff;
phi_max = cand(1)*d2r; climb = cand(3)*sin(cand(2)*d2r); Va_c = cand(3);
N = round(period/dt);
wm = 0; wh = zeros(4500, 1); wrs = 0; wppk = 0;
P0 = P_cum; H0 = ps.m_H2; Eb0 = ps.E_bat + ps.E_sc_bus; bd0 = ps.bat_dmg; fd0 = ps.fc_dP; t0 = t;
crash = false; task_end = 0; ru = 0; gd = F.gd; al = F.alpha; Pel = F.Pel; po = F.po;
for k = 1:N
    q = x(I.q); R = quat_utils('dcm', q); eul = quat_utils('to_euler', q);
    vned = R*x(I.vel);
    w = dryden_step(w);
    w.mean_ned = w.mean_ned + dt/20*(F.Wt - w.mean_ned);   % gio trung binh doi dan, tau 20 s (GT)
    wv = w.mean_ned; gb = w.gust_b;
    Va = norm(x(I.vel) - R.'*wv - gb);
    chi = atan2(vned(2), vned(1)); h = -x(I.pos(3));
    [gd, tk] = guidance_step(tk, x(1:2), chi, h, norm(vned(1:2)), eul(3));
    hc = hc + sign(tk.h_t - hc)*min(climb*dt, abs(tk.h_t - hc));
    Vc = min(max(Va, 22), 28);
    if Vc <= 25, a = (Vc - 22)/3; f = ffv(1, :)*(1 - a) + ffv(2, :)*a;
    else,        a = (Vc - 25)/3; f = ffv(2, :)*(1 - a) + ffv(3, :)*a; end
    ap.trim.de = f(1); ap.trim.dt = f(2); ap.trim.theta = f(3);
    meas = struct('phi', eul(1), 'theta', eul(2), 'p', x(I.om(1)), 'q', x(I.om(2)), ...
                  'r', x(I.om(3)), 'chi', chi, 'h', h, 'Va', Va);
    ref = struct('chi_c', gd.chi_c, 'h_c', hc, 'Va_c', Va_c, 'phi_ff', gd.phi_ff);
    [c, ast] = autopilot_step(ap, ast, meas, ref, phi_max, dt);
    bufS = [bufS(:, 2:end), c(1:3)]; bufT = [bufT(2:end), c(4)];
    ca = [bufS(:, 1); bufT(1)];

    [k1, aux] = ac_derivatives(x, ca, Ub, wv, gb, p);
    k2 = ac_derivatives(x + 0.5*dt*k1, ca, Ub, wv, gb, p);
    k3 = ac_derivatives(x + 0.5*dt*k2, ca, Ub, wv, gb, p);
    k4 = ac_derivatives(x + dt*k3, ca, Ub, wv, gb, p);
    Pel = aux.prop.Pel;
    [ps, po] = ps_step(pp, ps, Pel, dt, ems);
    Ub = po.V_bus;

    % Chi so co cau chap hanh
    us = max(abs(c(1:3))./amax);
    if any(abs(x(I.ddlt)) >= arate*(1 - 1e-6)), rs_run = rs_run + dt; else, rs_run = 0; end
    al = aux.alpha;
    bi = min(max(floor((al/d2r + 30)/0.02) + 1, 1), 4500);
    wm = max(wm, us); wh(bi) = wh(bi) + 1; wrs = max(wrs, rs_run); wppk = max(wppk, Pel);
    ta.m = max(ta.m, us); ta.hist(bi) = ta.hist(bi) + 1; ta.rs = max(ta.rs, rs_run);
    ta.xt2 = ta.xt2 + gd.e_py^2; ta.n = ta.n + 1;
    if us > 1 || al > alim || rs_run > 0.2
        viol_time = viol_time + dt; ta.viol = ta.viol + dt;
    end
    P_cum = P_cum + r_prev - gd.r; r_prev = gd.r;
    Ppk_f = max(Ppk_f, Pel);
    ru = max(ru, us);
    if mod(k, 10) == 0, iring = mod(iring, 60) + 1; ring(iring) = ru; ru = 0; end

    x = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
    x(I.q) = x(I.q)/norm(x(I.q));
    x(I.dlt) = min(max(x(I.dlt), -amax), amax);
    x(I.ddlt) = min(max(x(I.ddlt), -arate), arate);
    t = t + dt;
    if any(~isfinite(x)) || -x(I.pos(3)) < 5
        crash = true; F.stop = true; F.reason = 'crash'; break
    end
    if ps.m_H2 >= F.H2_avail, F.stop = true; F.reason = 'h2'; break, end
    if 1 - ps.it/pp.bat.cell.Q < fl.soc_stop, F.stop = true; F.reason = 'soc'; break, end
    if t >= fl.T_max, F.stop = true; F.reason = 'tmax'; break, end
    if gd.done, task_end = 1; break, end
    if t - F.t_task0 > tk.T_abort, task_end = 2; break, end
end
F.x = x; F.bufS = bufS; F.bufT = bufT; F.ast = ast; F.ps = ps; F.w = w; F.Ub = Ub;
F.task = tk; F.ta = ta; F.h_c = hc; F.t = t; F.rs_run = rs_run; F.P_cum = P_cum;
F.r_prev = r_prev; F.ring = ring; F.iring = iring; F.viol_time = viol_time; F.P_peak = Ppk_f;
F.gd = gd; F.alpha = al; F.Pel = Pel; F.po = po;

% ---- Nhan cua so
dur = max(t - t0, dt);
a99 = hist_p99(wh)/(alim/d2r);
mw = max([wm, a99, wrs/0.2]);
if crash, mw = 2; end
lab = [dur, mw, wm, a99, wrs/0.2, (P_cum - P0)/dur, (ps.m_H2 - H0)/dur, ...
       (ps.E_bat + ps.E_sc_bus - Eb0)/dur, (ps.bat_dmg - bd0)/dur, (ps.fc_dP - fd0)/dur, wppk, crash];
if strcmp(F.pol.kind, 'explore')
    F.WN(end+1, :) = single([fl.id, F.itask, Xw, lab]);
else
    F.DL(end+1, :) = single([t0, F.itask, tk.type, it, max(fcv, 0)*~isnan(fcv), info.ub, info.J, mw, dur, info.nfeas]);
end

% ---- Ket thuc nhiem vu
if task_end > 0 || F.stop
    F = finish_task(F, task_end);
    F.need_task = ~F.stop;
end
end

%% ------------------------------------------------------------------------------------
function F = start_task(F)
fl = F.fl;
F.itask = F.itask + 1;
sp = F.specs(mod(F.itask - 1, numel(F.specs)) + 1);
k = flight_kin(F);
tk = task_make(sp, F.x(1:2), k.chi, k.h);
F.Wt = -sp.W*[cosd(sp.dir); sind(sp.dir); 0];
F.turb_now = sp.turb*tk.turb_mult;
F.w = dryden_retune(F.w, sp.W, fl.h0, F.turb_now);
[gd, tk] = guidance_step(tk, F.x(1:2), k.chi, k.h, k.Vg, k.eul(3));
F.task = tk; F.gd = gd; F.r_prev = gd.r; F.t_task0 = F.t;
F.ta = struct('m', 0, 'hist', zeros(4500, 1), 'rs', 0, 'H0', F.ps.m_H2, ...
              'Eb0', F.ps.E_bat + F.ps.E_sc_bus, 'xt2', 0, 'n', 0, 'viol', 0);
F.need_task = false;
end

function F = finish_task(F, task_end)
tk = F.task; ta = F.ta; d2r = pi/180;
t_used = F.t - F.t_task0;
m_task = max([ta.m, hist_p99(ta.hist)/(F.p.alpha_lim/d2r), ta.rs/0.2]);
if strcmp(F.reason, 'crash'), m_task = 2; end
done = task_end == 1; aborted = task_end == 2;
ok = done && t_used <= tk.deadline && m_task <= 1;
Wn = norm(F.Wt(1:2));
F.TK(end+1, :) = [F.itask, tk.type, tk.lambda, tk.T_ref, tk.deadline, t_used, done, aborted, ok, ...
                  m_task, ta.viol, (F.ps.m_H2 - ta.H0)*1e3, (F.ps.E_bat + F.ps.E_sc_bus - ta.Eb0)/1e3, ...
                  sqrt(ta.xt2/max(ta.n, 1)), Wn, F.turb_now, F.stop && ~done];
end

function k = flight_kin(F)
I = F.I; x = F.x;
q = x(I.q); R = quat_utils('dcm', q); k.eul = quat_utils('to_euler', q);
vned = R*x(I.vel);
k.chi = atan2(vned(2), vned(1)); k.h = -x(I.pos(3)); k.Vg = norm(vned(1:2));
k.Va = norm(x(I.vel) - R.'*F.w.mean_ned - F.w.gust_b);
k.gamma = asin(max(min(-vned(3)/max(norm(vned), 1), 1), -1));
end

function ctx = make_ctx(F)
k = flight_kin(F); gd = F.gd; tk = F.task; p = F.p; pp = F.pp; fl = F.fl;
ctx.task_type = tk.type;
ctx.e_chi = abs(mod(gd.chi_c - k.chi + pi, 2*pi) - pi);
ctx.dh_rem = tk.h_t - k.h; ctx.e_py = abs(gd.e_py); ctx.kappa = gd.kappa;
ctx.d_wp = min(gd.d_wp, 3000); ctx.turn_next = gd.turn_next;
ctx.mass = p.mass.m; ctx.x_cg = p.mass.r_cg(1);
ctx.SM = -p.aero.Cma/p.aero.CLa + p.mass.r_cg(1)/p.geom.c; ctx.Jy = p.mass.J(2, 2);
ctx.P_fc = fl.P_fc; ctx.sc_use = double(pp.sc.use); ctx.passive = double(strcmp(fl.arch, 'passive'));
ctx.Td = fl.Td;
Wv = F.w.mean_ned; ctx.W = norm(Wv(1:2)); ctx.turb = F.turb_now;
rel = atan2(-Wv(2), -Wv(1)) - k.chi;                 % huong gio thoi tu, so voi huong bay
ctx.cos_rel = cos(rel); ctx.sin_rel = sin(rel);
ctx.Va = k.Va; ctx.phi = k.eul(1); ctx.gamma = k.gamma; ctx.alpha = F.alpha;
ctx.u_recent = max(F.ring); ctx.P_el = F.Pel;
ctx.soc = 1 - F.ps.it/pp.bat.cell.Q; ctx.esc = (F.ps.v_sc/pp.sc.Vmax)^2;
ctx.P_fcbus = F.po.P_fc; ctx.V_bus = F.Ub;
ctx.soc_ref = F.ps.soc0; ctx.E_bat_J = pp.bat.E_Wh*3600;
ctx.Va_allowed = F.C.Va(F.Va_ok);
ctx.P_fc_cap = pp.dc.eta*pp.fc.P_rated - pp.aux.P;
end

function a = hist_p99(h)
% Phan vi 99 cua alpha (deg) tu bieu do tan suat (o 0.02 deg tu -30 deg)
c = cumsum(h);
if c(end) == 0, a = 0; return; end
i = find(c >= 0.99*c(end), 1);
a = -30 + (i - 0.5)*0.02;
end

function res = flight_result(F)
fl = F.fl; pp = F.pp; ps = F.ps; TK = F.TK;
res = F.res0; res.ok = true;
res.msg = F.reason; res.stop_reason = F.reason;
res.t_flight = F.t;
res.n_tasks = size(TK, 1);
res.n_done = sum(TK(:, 7)); res.n_ok = sum(TK(:, 9)); res.n_abort = sum(TK(:, 8));
res.n_viol = sum(TK(:, 10) > 1);
res.ok_per_h = res.n_ok/max(F.t/3600, 1e-6);
res.viol_time = F.viol_time;
res.H2_avail_g = F.H2_avail*1e3; res.H2_used_g = ps.m_H2*1e3;
LHV = 120e6;
etaF = ps.E_fc_bus/max(ps.m_H2*LHV, eps);
soc_end = 1 - ps.it/pp.bat.cell.Q; esc_end = (ps.v_sc/pp.sc.Vmax)^2;
dE = (ps.soc0 - soc_end)*pp.bat.E_Wh*3600 + ...
     0.5*pp.sc.C*((sqrt(ps.esc0)*pp.sc.Vmax)^2 - ps.v_sc^2)*pp.sc.use;
res.H2_eq_g = (ps.m_H2 + dE/(max(etaF, 0.2)*LHV))*1e3;      % hieu chinh duy tri dien tich (posthoc_power)
res.H2_per_ok = res.H2_eq_g/max(res.n_ok, 1);
res.soc_end = soc_end; res.esc_end = esc_end;
res.bat_dmg = ps.bat_dmg;
fd = pp.fc.deg;
res.fc_uV = fd.k*(fd.load_uV*ps.fc_dP/(2*pp.fc.P_rated) + fd.idle_uV_h*ps.fc_idle_t/3600 + ...
                  fd.high_uV_h*ps.fc_high_t/3600);
res.fc_dP = ps.fc_dP; res.E_load_kJ = ps.E_load/1e3; res.P_mean = ps.E_load/max(F.t, eps);
res.P_peak = F.P_peak; res.bat_limit_t = ps.bat_limit;
res.n_dec = F.n_dec; res.wall_s = F.wall;
names_tk = {'itask','type','lambda','T_ref','deadline','t_used','done','aborted','ok', ...
            'm_task','viol_s','H2_g','Eb_kJ','rms_xtrack','W','turb','stopped'};
res.trace = struct('tasks', single(TK), 'task_names', {names_tk}, ...
                   'windows', F.WN, 'decisions', F.DL);
end
