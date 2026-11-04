function [res, F] = run_flight(fl, M, opts)
%RUN_FLIGHT Chuyen bay vong kin E3: bay chuoi nhiem vu den khi het H2, SoC < 30 % hoac roi.
%   [res, F] = RUN_FLIGHT(fl, M, opts)
%     fl    kich ban (flight_default); M bo mo hinh (e3_train) cho chinh sach 'model'
%     opts  ckpt_file: checkpoint giua chuyen bay ('' = khong luu); ckpt_every: chu ky luu
%           theo thoi gian tinh (s, mac dinh 300). Neu ckpt_file ton tai va dung chuyen
%           bay (id + chinh sach) thi chay tiep tu do.
%   Vong lap: dau moi nhiem vu va moi T_plan giay (tham do: giu kieu bay H giay), planner
%   chon kieu bay (goc nghieng cho phep, goc leo cho phep, van toc). Autopilot [R37, ch. 6]
%   bam duong [R37, ch. 10] trong cac gioi han do; 6-DOF + servo + he nguon trong vong.
%   Gain autopilot thiet ke tai 7 van toc 22-28 m/s, chon theo van toc lenh; feed-forward
%   trim noi suy theo van toc do (GT). Van toc nao khong trim duoc (khong hoi tu, lai vuot
%   gioi han, ga ngoai [0, 1] hoac alpha > alpha_lim) bi loai (R4); 25 m/s khong trim duoc
%   thi chuyen bay khong chay.
%   Gioi han cung (khong bao gio vuot): canh lai (servo), ga [0, 1], dien ap dong co
%   <= 44.4 V, FC/pin/sieu tu (ps_step). Khi pin cham gioi han xa, dong co chi nhan phan
%   cong suat con lai: tran dien ap Vcap giai tu cong suat cho phep o.P_motor_cap.
%   Muc vi pham cua mot cua so / nhiem vu (bao_cao/thiet_ke_E3.pdf, muc 5):
%     m = max(lenh lai/25 deg, alpha_p99/alpha_lim, bao hoa toc do lai lien tuc/0.2 s,
%             bao hoa dong co lien tuc/1 s, dong pin can/10C);  roi -> m = 2.
%   Nhiem vu dat = bay xong voi m <= 1 (khong co han). Dung khi: het H2 (fill x dung
%   tich), SoC < soc_stop (du tru ha canh 30 % [R27]), roi (h < 5 m hoac phan ky), hoac
%   T_max (gioi han ky thuat). Khoi luong H2 tieu thu bo qua trong dong luc hoc (GT).
%   Bam quy dao: quang duong bay (theo van toc dat) khi sai lech ngang |e_py| > e_max
%   (GT 20 m) -> nhan loff cua cua so, ti le lech duong cua nhiem vu (off_frac) va cua
%   chuyen bay (res.offtrack_frac).
%   Do phu tren quy dao cua chinh sach ('model'): sau moi quyet dinh mo mot cua so H
%   giay, do m va alpha_p99/alpha0 thuc te trong khi planner van quyet dinh lai moi
%   T_plan; so voi can tren R1, R2 cua kieu bay da chon -> res.trace.onpolicy.
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
nV = numel(C.Va);
aps = cell(1, nV); ff = zeros(nV, 3); trs = cell(1, nV); Va_ok = false(1, nV);
for iv = 1:nV
    tr = ac_trim(p, C.Va(iv), 0, struct('Ub', ps.V_bus));
    Va_ok(iv) = tr.feasible;
    if Va_ok(iv)
        aps{iv} = autopilot_design(p, tr); ff(iv, :) = [tr.de tr.dt tr.theta]; trs{iv} = tr;
    end
end
i25 = find(C.Va == 25);
if ~Va_ok(i25)
    res.msg = 'trim_failed_25'; return
end
% Van toc khong trim duoc: loai khoi cac kieu bay (R4); gain/feed-forward lay theo van
% toc trim duoc gan nhat (chi dung khi do noi suy theo van toc do).
okv = find(Va_ok);
for iv = find(~Va_ok)
    [~, j] = min(abs(okv - iv)); aps{iv} = aps{okv(j)}; ff(iv, :) = ff(okv(j), :);
end
blk = i25;                                   % khoang van toc trim duoc lien tuc quanh 25 m/s
while blk(1) > 1 && Va_ok(blk(1) - 1), blk = [blk(1) - 1, blk]; end
while blk(end) < nV && Va_ok(blk(end) + 1), blk = [blk, blk(end) + 1]; end
res.n_Va_ok = nnz(Va_ok);
F.Va_ok = Va_ok; F.Va_range = C.Va(blk([1 end]));
F.fl = fl; F.pol = pol; F.p = p; F.pp = pp; F.C = C; F.aps = aps; F.ff = ff;
F.I = ac_state_index();
F.ems = str2func(['ems_' pol.ems]);
F.specs = task_specs(fl.seed, fl.W_lvl);
F.H2_avail = fl.fill*pp.fc.m_H2_cap;
F.cruise = cruise_rate(trs{i25}, pp);       % suat tieu hao H2 hanh trinh (bang tinh nang)
res.mdot_ht_mg = F.cruise.mdot*1e6;
F.res0 = res;
F.x = trs{i25}.x0; F.x(F.I.pos) = [0; 0; -fl.h0];
F.ps = ps_init(pp, fl.soc0, fl.esc0, trs{i25}.Pel);
nS = round(p.act.delay/fl.dt); nT = round(p.act.thr_delay/fl.dt);
F.bufS = repmat(trs{i25}.cmd(1:3), 1, nS + 1); F.bufT = repmat(trs{i25}.cmd(4), 1, nT + 1);
F.ast = [];
sp = F.specs(1);
F.w = dryden_init(sp.W, sp.dir, fl.h0, sp.turb, 25, fl.dt, fl.wind_seed);
F.Wt = F.w.mean_ned; F.turb_now = sp.turb;
F.Ub = F.ps.V_bus;
F.rs = RandStream('mt19937ar', 'Seed', 9e4 + fl.id);
F.t = 0; F.h_c = fl.h0;
F.cand = C.nominal_traj; F.iVa = i25;
F.itask = 0; F.need_task = true; F.task = []; F.t_task0 = 0; F.ta = [];
F.gd = []; F.r_prev = 0; F.P_cum = 0;
F.ring = zeros(1, 60); F.iring = 0;
F.run = struct('rate', 0, 'mot', 0);
F.alpha = trs{i25}.alpha; F.Pel = trs{i25}.Pel;
F.po = struct('P_fc', trs{i25}.Pel, 'V_bus', F.ps.V_bus, 'P_motor_cap', inf);
F.lim = struct('surf_angle', 0, 'surf_rate', 0, 'motor', 0);
F.u2_int = 0; F.loff_tot = 0; F.dist_tot = 0;
F.stop = false; F.reason = '';
F.TK = zeros(0, 20); F.WN = zeros(0, 2 + numel(e3_feature_names()) + 21, 'single');
F.DL = zeros(0, 13, 'single');
% Cua so tren quy dao cua chinh sach (chi chinh sach 'model')
F.track = strcmp(pol.kind, 'model');
F.ow = struct('meta', zeros(0, 11), 'mx', zeros(0, 5), 'h', zeros(4500, 0));
F.OP = zeros(0, 20, 'single');
F.viol_time = 0; F.n_dec = 0; F.n_level = [0 0 0]; F.P_peak = 0; F.wall = 0;
F.ok = true; res.ok = true;
end

%% ------------------------------------------------------------------------------------
function F = flight_segment(F, M)
fl = F.fl; dt = fl.dt; I = F.I; d2r = pi/180;
if F.need_task, F = start_task(F); end

% ---- Quyet dinh
ctx = make_ctx(F);
[traj, it, info] = planner_decide(F.pol, M, ctx, F.C, F.rs);
F.cand = traj; [~, F.iVa] = min(abs(F.C.Va - traj(3)));
F.n_dec = F.n_dec + 1;
if strcmp(F.pol.kind, 'model'), F.n_level(info.level + 1) = F.n_level(info.level + 1) + 1; end
if strcmp(F.pol.kind, 'explore'), period = fl.H; else, period = fl.T_plan; end
Xw = e3_features(ctx, traj);

% ---- Mo phong mot cua so
x = F.x; bufS = F.bufS; bufT = F.bufT; ast = F.ast; ps = F.ps; w = F.w; Ub = F.Ub;
tk = F.task; ta = F.ta; hc = F.h_c; t = F.t; run = F.run; P_cum = F.P_cum;
r_prev = F.r_prev; ring = F.ring; iring = F.iring; viol_time = F.viol_time; Ppk_f = F.P_peak;
lim = F.lim; u2_int = F.u2_int; po = F.po; loff_tot = F.loff_tot; dist_tot = F.dist_tot;
p = F.p; pp = F.pp; ems = F.ems; Vmax = p.prop.Vmax;
amax = p.act.max; arate = p.act.rate; alim = p.alpha_lim; a0 = p.aero.alpha0;
Idis = pp.bat.I_dis_max;
ap = F.aps{F.iVa}; ffv = F.ff; Vg = F.C.Va;
phi_max = traj(1)*d2r; climb = traj(3)*sin(traj(2)*d2r); Va_c = traj(3);
N = round(period/dt);
wm = zeros(1, 5); wh = zeros(4500, 1); u2w = 0; lo = 0;
P0 = P_cum; H0 = ps.m_H2; Pb0 = ps.E_bat + ps.E_sc_bus; Pbat0 = ps.E_bat;
bd0 = ps.bat_dmg; fd0 = [ps.fc_dP, ps.fc_idle_t, ps.fc_high_t]; t0 = t;
crash = false; task_end = 0; ru = 0; gd = F.gd; al = F.alpha; Pel = F.Pel;
track = F.track;
if track
    F.ow.meta(end+1, :) = [t, F.itask, tk.type, it, info.u, info.q, info.a, info.aq, ...
                           info.pvp, info.grp, info.level];
    F.ow.mx(end+1, :) = 0; F.ow.h(:, end+1) = 0;
end
owmx = F.ow.mx; owh = F.ow.h; ow_t0 = F.ow.meta(:, 1);
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
    f = interp1(Vg, ffv, min(max(Va, Vg(1)), Vg(end)));
    ap.trim.de = f(1); ap.trim.dt = f(2); ap.trim.theta = f(3);
    meas = struct('phi', eul(1), 'theta', eul(2), 'p', x(I.om(1)), 'q', x(I.om(2)), ...
                  'r', x(I.om(3)), 'chi', chi, 'h', h, 'Va', Va);
    ref = struct('chi_c', gd.chi_c, 'h_c', hc, 'Va_c', Va_c, 'phi_ff', gd.phi_ff);
    [c, ast] = autopilot_step(ap, ast, meas, ref, phi_max, dt);
    bufS = [bufS(:, 2:end), c(1:3)]; bufT = [bufT(2:end), c(4)];
    ca = [bufS(:, 1); bufT(1)];

    % Tran dien ap dong co: <= 44.4 V; khi he nguon khong du cong suat, dien ap sao cho
    % cong suat dong co <= o.P_motor_cap cua buoc truoc (pin o gioi han xa).
    Vin_cmd = max(x(I.thr), 0)*Ub;
    pu = ac_propulsion(Va, x(I.thr), Ub, p, Vmax);
    Vcap = Vmax;
    if pu.Pel > po.P_motor_cap
        Vcap = motor_vcap(Va, min(Vin_cmd, Vmax), po.P_motor_cap, p);
    end

    [k1, aux] = ac_derivatives(x, ca, Ub, wv, gb, p, Vcap);
    k2 = ac_derivatives(x + 0.5*dt*k1, ca, Ub, wv, gb, p, Vcap);
    k3 = ac_derivatives(x + 0.5*dt*k2, ca, Ub, wv, gb, p, Vcap);
    k4 = ac_derivatives(x + dt*k3, ca, Ub, wv, gb, p, Vcap);
    Pel = aux.prop.Pel;
    [ps, po] = ps_step(pp, ps, Pel, dt, ems);

    % Muc vi pham, tung buoc
    m1 = max(abs(c(1:3))./amax);                                   % (1) lenh lai
    sat_rate = any(abs(x(I.ddlt)) >= arate*(1 - 1e-6));
    if sat_rate, run.rate = run.rate + dt; else, run.rate = 0; end   % (3)
    sat_mot = ca(4) >= 1 - 1e-9 || ca(4)*Ub >= min([Ub, Vmax, Vcap]) - 1e-9;
    if sat_mot, run.mot = run.mot + dt; else, run.mot = 0; end      % (4)
    Pb_need = pu.Pel + pp.aux.P - po.P_fc - po.P_sc;                % (5) theo nhu cau dong co
    m5 = max(bat_need(Pb_need, po.E, po.Rp), 0)/Idis;
    mi = [m1, 0, run.rate/0.2, run.mot/1, m5];
    al = aux.alpha;
    bi = min(max(floor((al/d2r + 30)/0.02) + 1, 1), 4500);
    u2 = 0.5*(mean((x(I.dlt)./amax).^2) + mean((x(I.ddlt)./arate).^2));
    wm = max(wm, mi); wh(bi) = wh(bi) + 1; u2w = u2w + u2*dt; u2_int = u2_int + u2*dt;
    ta.mx = max(ta.mx, mi); ta.hist(bi) = ta.hist(bi) + 1;
    ta.xt2 = ta.xt2 + gd.e_py^2; ta.n = ta.n + 1;
    ds = norm(vned(1:2))*dt; doff = ds*(abs(gd.e_py) > fl.e_max);  % quang duong lech duong
    lo = lo + doff; ta.loff = ta.loff + doff; ta.dist = ta.dist + ds;
    loff_tot = loff_tot + doff; dist_tot = dist_tot + ds;
    if any(mi > 1) || al > alim
        viol_time = viol_time + dt; ta.viol = ta.viol + dt;
    end
    if any(abs(x(I.dlt)) >= amax*(1 - 1e-6)), lim.surf_angle = lim.surf_angle + dt; end
    if sat_rate, lim.surf_rate = lim.surf_rate + dt; end
    if sat_mot, lim.motor = lim.motor + dt; end
    P_cum = P_cum + r_prev - gd.r; r_prev = gd.r;
    Ppk_f = max(Ppk_f, Pel);
    ru = max(ru, m1);
    if mod(k, 10) == 0, iring = mod(iring, 60) + 1; ring(iring) = ru; ru = 0; end
    if track
        owmx = max(owmx, repmat(mi, size(owmx, 1), 1)); owh(bi, :) = owh(bi, :) + 1;
    end

    x = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
    x(I.q) = x(I.q)/norm(x(I.q));
    x(I.dlt) = min(max(x(I.dlt), -amax), amax);
    x(I.ddlt) = min(max(x(I.ddlt), -arate), arate);
    x(I.thr) = min(max(x(I.thr), 0), 1);
    Ub = po.V_bus;
    t = t + dt;
    if track && ~isempty(ow_t0) && t - ow_t0(1) >= fl.H - 1e-9
        [F, owmx, owh, ow_t0] = close_windows(F, 1, owmx, owh, ow_t0, t, false);
    end
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
F.task = tk; F.ta = ta; F.h_c = hc; F.t = t; F.run = run; F.P_cum = P_cum;
F.r_prev = r_prev; F.ring = ring; F.iring = iring; F.viol_time = viol_time; F.P_peak = Ppk_f;
F.gd = gd; F.alpha = al; F.Pel = Pel; F.po = po; F.lim = lim; F.u2_int = u2_int;
F.loff_tot = loff_tot; F.dist_tot = dist_tot;
if track && F.stop && ~isempty(ow_t0)              % dung chuyen bay: dong moi cua so (bi cat)
    [F, owmx, owh] = close_windows(F, 1:numel(ow_t0), owmx, owh, ow_t0, t, crash);
end
F.ow.mx = owmx; F.ow.h = owh;

% ---- Nhan cua so
dur = max(t - t0, dt);
p99 = hist_p99(wh);
mc = wm; mc(2) = p99/(alim/d2r);
mw = max(mc); if crash, mw = 2; end
fcuv = fc_uV(pp, ps.fc_dP - fd0(1), ps.fc_idle_t - fd0(2), ps.fc_high_t - fd0(3));
lab = [dur, mw, mc, p99/(a0/d2r), (P_cum - P0)/dur, (ps.m_H2 - H0)/dur, ...
       (ps.E_bat + ps.E_sc_bus - Pb0)/dur, (ps.E_bat - Pbat0)/dur, u2w/dur, lo/dur, ...
       (ps.bat_dmg - bd0)/dur, fcuv/dur, crash, ...
       F.cruise.mdot, ps.soc0, tk.L, ta.violated];
if strcmp(F.pol.kind, 'explore')
    F.WN(end+1, :) = single([fl.id, F.itask, Xw, lab]);
else
    F.DL(end+1, :) = single([t0, F.itask, tk.type, it, info.u, info.a, info.pvp, info.J, ...
                             info.nfeas, info.level, mw, dur, traj(3)]);
end

% ---- Ket thuc nhiem vu
ta.violated = ta.violated || task_m(F.ta, alim) > 1;
F.ta = ta;
if task_end > 0 || F.stop
    F = finish_task(F, task_end);
    F.need_task = ~F.stop;
end
end

function [F, owmx, owh, ow_t0] = close_windows(F, idx, owmx, owh, ow_t0, t, crash)
% Dong cac cua so tren quy dao chinh sach; ghi mot dong vao F.OP:
% [t0 itask loai it u q a aq pvp nhom muc | m m1 m2 m3 m4 m5 a_thuc dur roi]
d2r = pi/180; alim = F.p.alpha_lim/d2r; a0 = F.p.aero.alpha0/d2r;
for j = idx(:).'
    p99 = hist_p99(owh(:, j));
    mc = owmx(j, :); mc(2) = p99/alim;
    m = max(mc); if crash, m = 2; end
    F.OP(end+1, :) = single([F.ow.meta(j, :), m, mc, p99/a0, t - ow_t0(j), crash]);
end
keep = true(numel(ow_t0), 1); keep(idx) = false;
owmx = owmx(keep, :); owh = owh(:, keep); ow_t0 = ow_t0(keep);
F.ow.meta = F.ow.meta(keep, :);
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
F.ta = struct('mx', zeros(1, 5), 'hist', zeros(4500, 1), 'H0', F.ps.m_H2, ...
              'Eb0', F.ps.E_bat + F.ps.E_sc_bus, 'xt2', 0, 'n', 0, 'viol', 0, 'violated', false, ...
              'loff', 0, 'dist', 0);
F.need_task = false;
end

function m = task_m(ta, alim)
mc = ta.mx; mc(2) = hist_p99(ta.hist)/(alim*180/pi);
m = max(mc);
end

function F = finish_task(F, task_end)
tk = F.task; ta = F.ta; d2r = pi/180;
t_used = F.t - F.t_task0;
mc = ta.mx; mc(2) = hist_p99(ta.hist)/(F.p.alpha_lim/d2r);
m_task = max(mc);
if strcmp(F.reason, 'crash'), m_task = 2; end
done = task_end == 1; aborted = task_end == 2;
ok = done && m_task <= 1;
Wn = norm(F.Wt(1:2));
F.TK(end+1, :) = [F.itask, tk.type, tk.T_dn, t_used, done, aborted, ok, m_task, mc, ...
                  ta.viol, (F.ps.m_H2 - ta.H0)*1e3, (F.ps.E_bat + F.ps.E_sc_bus - ta.Eb0)/1e3, ...
                  sqrt(ta.xt2/max(ta.n, 1)), ta.loff/max(ta.dist, eps), Wn, F.stop && ~done];
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
ctx.sc_use = double(pp.sc.use); ctx.Td = fl.Td; ctx.f_c = fl.f_c;
Wv = F.w.mean_ned; ctx.W = norm(Wv(1:2)); ctx.turb = F.turb_now;
rel = atan2(-Wv(2), -Wv(1)) - k.chi;                 % huong gio thoi tu, so voi huong bay
ctx.cos_rel = cos(rel); ctx.sin_rel = sin(rel);
ctx.Va = k.Va; ctx.phi = k.eul(1); ctx.gamma = k.gamma; ctx.alpha = F.alpha;
ctx.u_recent = max(F.ring); ctx.P_el = F.Pel;
ctx.soc = 1 - F.ps.it/pp.bat.cell.Q; ctx.esc = (F.ps.v_sc/pp.sc.Vmax)^2;
ctx.P_fcbus = F.po.P_fc; ctx.V_bus = F.Ub;
ctx.soc_ref = F.ps.soc0; ctx.E_bat_J = pp.bat.E_Wh*3600;
ctx.Va_allowed = F.C.Va(F.Va_ok); ctx.Va_range = F.Va_range;
ctx.mdot_ht = F.cruise.mdot; ctx.f_ht = F.cruise.f;
ctx.L_nv = tk.L; ctx.task_violated = F.ta.violated;
ctx.dV_eol_uV = 0.1*pp.fc.Vcell_rated*1e6;           % het tuoi tho FC: sut 10 % [R16]
ctx.C_fc = 560*pp.fc.P_rated/1e3;                     % USD [R14]
ctx.C_bat = 720*pp.bat.E_Wh/1e3;                      % USD [R14]
end

function cr = cruise_rate(tr, pp)
% Suat tieu hao H2 tuong duong khi bay hanh trinh bang o 25 m/s, khong gio (bang tinh
% nang tinh truoc chuyen bay): FC cap tai trong gioi han dinh muc; phan vuot (neu co) do
% pin cap, quy ra H2 voi s_eq = s0 = 2.5 (dang ECMS [R30]).
fc = pp.fc; eta = pp.dc.eta;
P_dem = tr.Pel + pp.aux.P;
P_st = min(P_dem/eta, fc.P_rated);
lo = fc.Imin; hi = fc.I_rated;
for k = 1:60
    mid = 0.5*(lo + hi);
    [~, va] = fc_voltage(mid, 0, fc);
    if fc_voltage(mid, va, fc)*mid > P_st, hi = mid; else, lo = mid; end
end
etaF = fc.eta_F_hi; if P_st < fc.P_eta_switch, etaF = fc.eta_F_lo; end
mdot = fc.Ncell*lo/(2*pp.F)*pp.M_H2/etaF + 2.5*max(P_dem - eta*fc.P_rated, 0)/120e6;
cr = struct('mdot', mdot, 'f', mdot/25, 'P_dem', P_dem);
end

function Vc = motor_vcap(Va, Vhi, Pcap, p)
% Dien ap vao dong co lon nhat sao cho cong suat dien <= Pcap (cong suat tang theo Vin)
lo = 0; hi = max(Vhi, 0);
for k = 1:40
    mid = 0.5*(lo + hi);
    o = ac_propulsion(Va, 1, mid, p, mid);
    if o.Pel > Pcap, hi = mid; else, lo = mid; end
end
Vc = lo;
end

function I = bat_need(P, E, Rp)
% Dong pin can de cap cong suat P (khong gioi han); vuot cong suat cuc dai -> E/(2 Rp)
disc = E^2 - 4*Rp*P;
if disc < 0, I = E/(2*Rp); else, I = (E - sqrt(disc))/(2*Rp); end
end

function uV = fc_uV(pp, dP, t_idle, t_high)
% Sut ap moi cell (uV) [R16][R17]: thay doi tai, chay khong tai, tai cao
fd = pp.fc.deg;
uV = fd.k*(fd.load_uV*dP/(2*pp.fc.P_rated) + fd.idle_uV_h*t_idle/3600 + fd.high_uV_h*t_high/3600);
end

function a = hist_p99(h)
% Phan vi 99 cua alpha (deg) tu bieu do tan suat (o 0.02 deg tu -30 deg)
c = cumsum(h);
if c(end) == 0, a = 0; return; end
i = find(c >= 0.99*c(end), 1);
a = -30 + (i - 0.5)*0.02;
end

function res = flight_result(F)
pp = F.pp; ps = F.ps; TK = F.TK;
res = F.res0; res.ok = true;
res.msg = F.reason; res.stop_reason = F.reason;
res.t_flight = F.t;
res.n_tasks = size(TK, 1);
res.n_done = sum(TK(:, 5)); res.n_ok = sum(TK(:, 7)); res.n_abort = sum(TK(:, 6));
res.n_viol = sum(TK(:, 8) > 1);
okt = TK(:, 7) == 1;
res.t_per_ok = mean(TK(okt, 4)); if ~any(okt), res.t_per_ok = NaN; end
res.ok_per_h = res.n_ok/max(F.t/3600, 1e-6);
res.viol_time = F.viol_time;
res.H2_avail_g = F.H2_avail*1e3; res.H2_used_g = ps.m_H2*1e3;
LHV = 120e6;
etaF = ps.E_fc_bus/max(ps.m_H2*LHV, eps);
soc_end = 1 - ps.it/pp.bat.cell.Q; esc_end = (ps.v_sc/pp.sc.Vmax)^2;
dE = (ps.soc0 - soc_end)*pp.bat.E_Wh*3600 + ...
     0.5*pp.sc.C*((sqrt(ps.esc0)*pp.sc.Vmax)^2 - ps.v_sc^2)*pp.sc.use;
res.H2_eq_g = (ps.m_H2 + dE/(max(etaF, 0.2)*LHV))*1e3;      % hieu chinh duy tri dien tich, dang ECMS [R30]
res.H2_per_ok = res.H2_eq_g/max(res.n_ok, 1);
res.soc_end = soc_end; res.esc_end = esc_end;
res.bat_dmg = ps.bat_dmg;
res.fc_uV = fc_uV(pp, ps.fc_dP, ps.fc_idle_t, ps.fc_high_t);
% Chi phi van hanh (USD) theo gia [R14]: H2 6.9 USD/kg; FC 560 USD/kW het tuoi tho khi
% sut 10 % dien ap [R16]; pin 720 USD/kWh het tuoi tho khi mat 20 % dung luong.
res.usd_h2 = 6.9*res.H2_eq_g/1e3;
res.usd_fc = 560*pp.fc.P_rated/1e3*res.fc_uV/(0.1*pp.fc.Vcell_rated*1e6);
res.usd_bat = 720*pp.bat.E_Wh/1e3*res.bat_dmg/20;
res.usd = res.usd_h2 + res.usd_fc + res.usd_bat;
res.usd_per_ok = res.usd/max(res.n_ok, 1);
res.u2_mean = F.u2_int/max(F.t, eps);
res.offtrack_frac = F.loff_tot/max(F.dist_tot, eps);
res.fc_dP = ps.fc_dP; res.E_load_kJ = ps.E_load/1e3; res.P_mean = ps.E_load/max(F.t, eps);
res.P_peak = F.P_peak;
L = ps.lim;
res.lim_surf_angle = F.lim.surf_angle; res.lim_surf_rate = F.lim.surf_rate;
res.lim_motor = F.lim.motor; res.lim_fc_ramp = L.fc_ramp; res.lim_fc_pwr = L.fc_pwr;
res.lim_sc = L.sc; res.lim_bat_dis = L.bat_dis; res.lim_bat_chg = L.bat_chg;
res.lim_bat_full = L.bat_full; res.lim_dump = L.dump;
res.E_short_kJ = ps.E_short/1e3; res.E_dump_kJ = ps.E_dump/1e3;
res.n_dec = F.n_dec; res.n_lvl0 = F.n_level(1); res.n_lvl1 = F.n_level(2); res.n_lvl2 = F.n_level(3);
res.wall_s = F.wall;
names_tk = {'itask','type','T_dn','t_used','done','aborted','ok','m_task', ...
            'm1','m2','m3','m4','m5','viol_s','H2_g','Eb_kJ','rms_xtrack','off_frac','W','stopped'};
names_wn = [{'id','itask'}, e3_feature_names(), {'dur','m','m1','m2','m3','m4','m5','a', ...
            'prog','h2','pb','pbat','u2','loff','bd','fcuv','crash','mdot_ht','soc_ref','L_nv', ...
            'task_viol'}];
names_dl = {'t0','itask','type','it','u','a','pvp','J','nfeas','level','m','dur','Va'};
names_op = {'t0','itask','type','it','u','q','a','aq','pvp','grp','level', ...
            'm','m1','m2','m3','m4','m5','a_real','dur','crash'};
res.trace = struct('tasks', single(TK), 'task_names', {names_tk}, ...
                   'windows', F.WN, 'window_names', {names_wn}, ...
                   'decisions', F.DL, 'decision_names', {names_dl}, ...
                   'onpolicy', F.OP, 'onpolicy_names', {names_op});
end

function n = e3_feature_names()
[~, n] = e3_features();
end
