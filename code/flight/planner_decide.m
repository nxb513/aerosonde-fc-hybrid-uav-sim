function [traj, it, info] = planner_decide(pol, M, ctx, C, rs)
%PLANNER_DECIDE Chon kieu bay tai mot thoi diem quyet dinh (thiet ke E3, muc 6-7).
%   [traj, it, info] = PLANNER_DECIDE(pol, M, ctx, C, rs)
%     traj  [phi_max gamma_max Va]; it chi so trong luoi C.traj (0 neu khong thuoc luoi)
%   Chinh sach 'model', voi H = 10 s:
%     J = J_nl + J_tg + J_bq + J_rr   (kg H2; bao cao bang gam)
%       J_nl = Delta M - f_ht d,  Delta M = H (mdot_H2 + s_eq P_b/LHV),
%              s_eq = s0 (1 + k (SoC_ref - SoC)) dang ECMS [R30]
%       J_tg = kt mdot_ht (H - d/V_ref)
%       J_bq = kb f_ht l_lech    (l_lech = quang duong du bao bay voi |e_py| > e_max trong
%              H giay: H2 dot ma khong bam duoc duong)
%       J_rr = p_vp f_ht L_nv     (H2 de bay lai nhiem vu; 0 neu nhiem vu da vi pham)
%     Rang buoc: R1 u_0.9 <= 1 (AFP + CQR [R32], eps = 0.1); R2 a_(1-delta) <= 1 (bien
%     stall + CQR); R3 SoC du bao >= 0.35; R4 van toc trim duoc (ctx.Va_allowed).
%     Khong kieu bay nao dat ca 4: bo R1; neu van khong co, chon a nho nhat (khong co R2
%     thi chon u nho nhat).
%   info: u, q, a, aq, pvp, J (kieu bay da chon), nfeas (so kieu bay dat ca 4), grp, level
%   (0: dat ca 4; 1: da bo R1; 2: chon an toan nhat).
info = struct('u', NaN, 'q', NaN, 'a', NaN, 'aq', NaN, 'pvp', NaN, 'J', NaN, ...
              'nfeas', NaN, 'grp', NaN, 'level', 0);
switch pol.kind
    case 'explore'
        r = C.range; v = ctx.Va_range;
        traj = [r(1,1) + diff(r(1,:))*rand(rs), r(2,1) + diff(r(2,:))*rand(rs), ...
                v(1) + diff(v)*rand(rs)];
        it = 0;
        return
    case 'fixed'
        traj = pol.traj; it = find(ismember(C.traj, traj, 'rows'), 1);
        if isempty(it), it = 0; end
        return
end
mdl = M.(pol.model); nm = M.norm; H = nm.H;
idx = find(ismember(C.traj(:, 3), ctx.Va_allowed));
TR = C.traj(idx, :);
X = e3_features(ctx, TR);
grp = wind_bin(ctx.W);

% R1: AFP (rung luong tu cua m) va xac suat vi pham
[q, pvp] = qrf_predict(mdl.afp, X(:, mdl.afp.cols), 0.9, 1);
switch pol.conf
    case 'group',  u = cqr_bound(mdl.C, q, grp*ones(size(q)));
    case 'global', u = cqr_bound(mdl.C, q);
    otherwise,     u = q;
end
% R2: bien stall o muc 1 - delta
S = mdl.stall.(delta_key(pol.delta));
aq = qrf_predict(mdl.stall.P, X(:, mdl.stall.P.cols), 1 - pol.delta);
if strcmp(pol.conf, 'group') && S.use_group
    a = cqr_bound(S.C, aq, grp*ones(size(aq)));
elseif strcmp(pol.conf, 'none')
    a = aq;
else
    a = cqr_bound(S.C, aq);
end

% Mo hinh thay the
prog = rf_predict(mdl.prog, X(:, mdl.prog.cols));
h2   = rf_predict(mdl.h2, X(:, mdl.h2.cols));
pb   = rf_predict(mdl.pb, X(:, mdl.pb.cols));
pbat = rf_predict(mdl.pbat, X(:, mdl.pbat.cols));
loff = H*max(rf_predict(mdl.loff, X(:, mdl.loff.cols)), 0);
d = H*max(prog, 0);
s_eq = nm.s0*(1 + nm.k_soc*(ctx.soc_ref - ctx.soc));
dM = H*(h2 + s_eq*pb/nm.LHV);
if pol.deg
    fcuv = max(rf_predict(mdl.fcuv, X(:, mdl.fcuv.cols)), 0);
    bd = max(rf_predict(mdl.bd, X(:, mdl.bd.cols)), 0);
    usd = H*(ctx.C_fc*fcuv/ctx.dV_eol_uV + ctx.C_bat*bd/20);
    dM = dM + usd/nm.c_H2;
end
Jnl = dM - ctx.f_ht*d;
Jtg = pol.kt*ctx.mdot_ht*(H - d/nm.V_ref);
Jbq = pol.kb*ctx.f_ht*loff;
Jrr = pvp*ctx.f_ht*ctx.L_nv*(pol.jrr && ~ctx.task_violated);
switch pol.obj
    case 'J',     J = Jnl + Jtg + Jbq + Jrr;
    case 'norm',  J = Jnl/nm.med(1) + Jtg/nm.med(2) + Jbq/nm.med(3) + Jrr/nm.med(4);
    case 'min_u', J = u;
end

% Rang buoc va thu tu noi
c1 = u <= 1 | ~pol.r1;
c2 = a <= 1 | ~pol.r2;
c3 = ctx.soc - pbat*H/ctx.E_bat_J >= nm.soc_floor;
f0 = c1 & c2 & c3;
info.nfeas = nnz(f0);
if any(f0)
    cand = find(f0); lvl = 0;
elseif any(c2 & c3)
    cand = find(c2 & c3); lvl = 1;
else
    cand = []; lvl = 2;
end
if lvl < 2
    [~, k] = min(J(cand)); j = cand(k);
elseif pol.r2
    [~, j] = min(a);
else
    [~, j] = min(u);
end
it = idx(j); traj = C.traj(it, :);
info.u = u(j); info.q = q(j); info.a = a(j); info.aq = aq(j); info.pvp = pvp(j);
info.J = J(j); info.grp = grp; info.level = lvl;
end

function b = wind_bin(W)
b = discretize(W, [-inf 3 7 10 inf]);
end

function k = delta_key(delta)
k = sprintf('d%04d', round(delta*1e4));
end
