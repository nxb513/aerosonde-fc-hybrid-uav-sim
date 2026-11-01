function [it, fcv, info] = planner_decide(pol, M, ctx, C, lam, rs)
%PLANNER_DECIDE Chon ung vien (quy dao + muc bu) tai mot thoi diem quyet dinh.
%   [it, fcv, info] = PLANNER_DECIDE(pol, M, ctx, C, lam, rs)
%     it   chi so quy dao trong C.traj; fcv tan so cat EMS (NaN = EMS mac dinh cua chinh sach)
%   Chinh sach 'model' (cua ta va ablation), voi du bao cho chan troi H = M.norm.H giay
%   (giu ung vien):
%     tau = 1 - tien_do/V_ref                       (thoi gian; V_ref = 25 m/s)
%     e   = (H2 + s_eq * P_pin_sieutu / LHV)/H2_ref  (H2 tuong duong, s_eq theo SoC nhu
%           ECMS [R30]: s_eq = s0 (1 + k (SoC_ref - SoC)); s0 = 2.5, k = 3 la GT)
%     d   = suy giam pin [R18] + bien thien cong suat FC [R16][R17], chuan hoa trung vi
%     J   = lam*tau + (1 - lam)*(e + d)
%   Rang buoc: can tren AFP (conformal [R32]) <= 1; SoC du bao cuoi chan troi >= 0.35
%   (du tru ha canh 30 % [R27] + 5 % bien, GT). Khong ung vien nao kha thi -> chon can
%   tren AFP nho nhat.
info = struct('ub', NaN, 'J', NaN, 'nfeas', NaN);
switch pol.kind
    case 'explore'
        allowed = find(ismember(C.traj(:, 3), ctx.Va_allowed));
        it = allowed(randi(rs, numel(allowed)));
        fcv = C.fc(randi(rs, numel(C.fc)));
        if ctx.passive, fcv = NaN; end
        return
    case 'fixed'
        it = find(ismember(C.traj, pol.traj, 'rows')); fcv = NaN;
        return
end
mdl = M.(pol.model); nm = M.norm;
nT = size(C.traj, 1);
if ctx.passive, fcs = 0; else, fcs = C.fc; end
nF = numel(fcs);
TR = repmat(C.traj, nF, 1); FC = kron(fcs(:), ones(nT, 1));
X = e3_features(ctx, TR, FC);
X1 = X(1:nT, :);
q = qrf_predict(mdl.afp, X1(:, mdl.afp.cols), mdl.tau);
switch pol.conf
    case 'group', ub = cqr_bound(mdl.C, q, wind_bin(ctx.W)*ones(nT, 1));
    case 'global', ub = cqr_bound(mdl.C, q);
    case 'none', ub = q;
    otherwise, ub = q;                       % 'off': khong rang buoc, chi de ghi log
end
prog = rf_predict(mdl.prog, X1(:, mdl.prog.cols));
ppk = rf_predict(mdl.ppk, X1(:, mdl.ppk.cols));
h2 = rf_predict(mdl.h2, X(:, mdl.h2.cols));
eb = rf_predict(mdl.eb, X(:, mdl.eb.cols));
dg = rf_predict(mdl.dg, X(:, mdl.dg.cols));
s_eq = nm.s0*(1 + nm.k_soc*(ctx.soc_ref - ctx.soc));
e = (h2 + s_eq*eb/nm.LHV)/nm.h2_ref;
tau = 1 - repmat(prog, nF, 1)/nm.V_ref;
J = lam*tau + (1 - lam)*(e + dg);
ubr = repmat(ub, nF, 1);
feas = ctx.soc - eb*nm.H/ctx.E_bat_J >= nm.soc_floor;
if ~strcmp(pol.conf, 'off'), feas = feas & ubr <= 1; end
mask = repmat(ismember(C.traj(:, 3), ctx.Va_allowed), nF, 1);   % bo van toc khong trim duoc
switch pol.mode
    case 'replan'     % FC theo tai (f_c cao nhat), loai quy dao can pin cho dinh cong suat
        mask = mask & FC == max(fcs);
        feas = feas & repmat(ppk <= ctx.P_fc_cap, nF, 1);
    case 'comp'       % giu quy dao danh nghia, chi chon muc bu
        mask = repmat((1:nT).' == C.nominal, nF, 1);
end
cand = find(mask & feas);
if ~isempty(cand)
    [~, k] = min(J(cand)); j = cand(k);
else
    cand = find(mask);
    [~, k] = min(ubr(cand) + 1e-6*J(cand)); j = cand(k);
end
it = mod(j - 1, nT) + 1;
fcv = FC(j); if fcv == 0, fcv = NaN; end
info.ub = ubr(j); info.J = J(j); info.nfeas = numel(find(mask & feas));
end

function b = wind_bin(W)
b = discretize(W, [-inf 3 7 10 inf]);
end
