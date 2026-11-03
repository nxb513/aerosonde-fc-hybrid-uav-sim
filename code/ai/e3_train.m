function [M, rep] = e3_train(D, opts)
%E3_TRAIN Huan luyen bo mo hinh cho planner E3 tu du lieu tham do (e3_dataset).
%   [M, rep] = E3_TRAIN(D, opts)
%   Chia du lieu theo chuyen bay (khong theo mau, tranh ro ri): 60 % huan luyen, 20 %
%   hieu chuan conformal, 20 % kiem tra (e3_split).
%   Moi bien the (full; A4 bo cau hinh; A5 bo gio; A6 bo trang thai dien) gom:
%     afp    rung hoi quy luong tu [R39] cua m (qrf_predict, tau = 0.9) + CQR mot phia
%            [R32] eps = 0.1, hieu chuan toan cuc va theo nhom muc gio (R1); xac suat vi
%            pham p_vp = P(m > 1) doc tu cung rung
%     stall  rung luong tu cua a = alpha_p99/alpha0; CQR o muc delta = 0.005 / 0.01 / 0.02
%            (R2). Hieu chuan theo nhom gio chi khi moi nhom co it nhat 20 mau vuot ky
%            vong (n_nhom * delta >= 20, GT); neu khong thi hieu chuan chung.
%     prog, h2, pb, pbat, u2, fcuv, bd   rung hoi quy (mo hinh thay the)
%   Sieu tham so rung (GT): 60 cay, la toi thieu 20 mau, so dac trung moi lan chia = 1/3
%   ([R39] dung 1000 cay, la > 10, mtry = p/3; ket qua it nhay voi mtry va co la).
%   M.norm.med: trung vi |J_nl|, |J_tg|, |J_dk|, |J_rr| tren tap hieu chuan (chinh sach C5).
%   rep.test: chi so tren tap kiem tra; rep.calib: do hieu chinh cua p_vp (bien the full).
if nargin < 2, opts = struct(); end
def = struct('ntrees', 60, 'leaf', 20, 'tau', 0.9, 'alpha', 0.1, 'deltas', [0.005 0.01 0.02], ...
             'min_exc', 20);
f = fieldnames(def);
for k = 1:numel(f), if ~isfield(opts, f{k}), opts.(f{k}) = def.(f{k}); end, end
if ~isfield(opts, 'variants'), opts.variants = {'full', 'A4', 'A5', 'A6'}; end
G = D.groups;
sp = e3_split(D.flight);
tr = sp.tr; ca = sp.ca; te = sp.te; ok = ~D.crash;

nm = struct('H', 10, 'V_ref', 25, 'LHV', 120e6, 's0', 2.5, 'k_soc', 3, 'soc_floor', 0.35, ...
            'c_H2', 6.9, 'tau', opts.tau, 'alpha', opts.alpha, 'med', [1 1 1 1]);
M.split = sp; M.names = D.names; M.opts = opts;

drop = struct('full', [], 'A4', G.config, 'A5', G.wind, 'A6', G.elec);
surr = {'prog', 'h2', 'pb', 'pbat', 'u2', 'fcuv', 'bd'};
rows = {};
for v = opts.variants
    cols = setdiff(1:size(D.X, 2), drop.(v{1}));
    fprintf('[e3_train] %s: %d dac trung, %d mau huan luyen\n', v{1}, numel(cols), nnz(tr));
    mdl = struct();
    mdl.afp = fit_forest(D.X(tr, cols), D.m(tr), opts, true); mdl.afp.cols = cols;
    qc = qrf_predict(mdl.afp, D.X(ca, cols), opts.tau);
    mdl.C = cqr_calibrate(qc, D.m(ca), opts.alpha, D.wbin(ca));
    mdl.stall.P = fit_forest(D.X(tr, cols), D.a(tr), opts, true); mdl.stall.P.cols = cols;
    for dl = opts.deltas
        aqc = qrf_predict(mdl.stall.P, D.X(ca, cols), 1 - dl);
        Cs = cqr_calibrate(aqc, D.a(ca), dl, D.wbin(ca));
        ng = arrayfun(@(g) nnz(D.wbin(ca) == g), Cs.groups);
        use_group = numel(ng) == 4 && all(ng*dl >= opts.min_exc) && all(isfinite(Cs.by_group));
        mdl.stall.(sprintf('d%04d', round(dl*1e4))) = struct('C', Cs, 'use_group', use_group, ...
            'n_group', ng);
    end
    r = tr & ok;
    for s = surr
        mdl.(s{1}) = fit_forest(D.X(r, cols), D.(s{1})(r), opts, false); mdl.(s{1}).cols = cols;
    end
    M.(v{1}) = mdl;

    % Danh gia tren tap kiem tra
    Xt = D.X(te, cols);
    [qt, pt] = qrf_predict(mdl.afp, Xt, opts.tau, 1);
    row = afp_metrics(qt, D.m(te), mdl.C, D.wbin(te), opts.tau);
    S = mdl.stall.d0100;
    aqt = qrf_predict(mdl.stall.P, Xt, 0.99);
    row.stall_use_group = S.use_group;
    row.stall_cov_global = mean(D.a(te) <= cqr_bound(S.C, aqt));
    if S.use_group
        row.stall_cov_group = mean(D.a(te) <= cqr_bound(S.C, aqt, D.wbin(te)));
    else
        row.stall_cov_group = NaN;
    end
    row.stall_exceed_rate = mean(D.a(te) > 1);
    row.stall_pinball = mean(max(0.99*(D.a(te) - aqt), (0.99 - 1)*(D.a(te) - aqt)));
    row.pvp_brier = mean((pt - D.viol(te)).^2);
    rt = te & ok;
    for s = surr
        row.(['R2_' s{1}]) = r2(rf_predict(mdl.(s{1}), D.X(rt, cols)), D.(s{1})(rt));
    end
    row.variant = string(v{1}); row.n_train = nnz(tr);
    rows{end+1} = row; %#ok<AGROW>
    if strcmp(v{1}, 'full'), rep.calib = calib_table(pt, D.viol(te)); end
end
rep.test = struct2table([rows{:}]');
rep.test = movevars(rep.test, {'variant', 'n_train'}, 'Before', 1);

% Trung vi cac khoan cua J tren tap hieu chuan (chinh sach C5), dung bo mo hinh full
mf = M.full; rc = ca & ok;
Xc = D.X(rc, :);
H = nm.H;
[~, pv] = qrf_predict(mf.afp, Xc(:, mf.afp.cols), opts.tau, 1);
prog = rf_predict(mf.prog, Xc(:, mf.prog.cols)); h2 = rf_predict(mf.h2, Xc(:, mf.h2.cols));
pb = rf_predict(mf.pb, Xc(:, mf.pb.cols)); u2 = min(max(rf_predict(mf.u2, Xc(:, mf.u2.cols)), 0), 1);
soc = Xc(:, strcmp(D.names, 'soc')); mdot = D.mdot_ht(rc); fht = mdot/nm.V_ref;
d = H*max(prog, 0);
s_eq = nm.s0*(1 + nm.k_soc*(D.soc_ref(rc) - soc));
Jt = [H*(h2 + s_eq.*pb/nm.LHV) - fht.*d, mdot.*(H - d/nm.V_ref), mdot*H.*u2, ...
      pv.*fht.*D.L_nv(rc).*~D.task_viol(rc)];
nm.med = max(median(abs(Jt), 1), eps);
M.norm = nm;
end

function P = fit_forest(X, y, opts, quant)
rng(42);
B = TreeBagger(opts.ntrees, X, y, 'Method', 'regression', 'MinLeafSize', opts.leaf, ...
    'NumPredictorsToSample', max(1, round(size(X, 2)/3)));
if quant, P = rf_pack(B, X, y); else, P = rf_pack(B); end
end

function v = r2(yh, y)
v = 1 - sum((y - yh).^2)/sum((y - mean(y)).^2);
end

function T = calib_table(p, y)
% Do hieu chinh cua xac suat du bao: 10 khoang deu tren [0, 1]
e = linspace(0, 1, 11); b = discretize(p, e, 'IncludedEdge', 'right'); b(p == 0) = 1;
rows = {};
for k = 1:10
    s = b == k;
    rows{end+1} = struct('bin_lo', e(k), 'bin_hi', e(k+1), 'n', nnz(s), ...
        'p_mean', mean(p(s)), 'freq', mean(y(s))); %#ok<AGROW>
end
T = struct2table([rows{:}]');
end
