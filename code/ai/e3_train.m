function [M, rep] = e3_train(D, opts)
%E3_TRAIN Huan luyen bo mo hinh cho planner E3 tu du lieu tham do (e3_dataset).
%   [M, rep] = E3_TRAIN(D, opts)
%   Chia du lieu theo chuyen bay (khong theo mau, tranh ro ri): 60 % huan luyen, 20 %
%   hieu chuan conformal, 20 % kiem tra (e3_split).
%   Moi bien the (full; A3 bo cau hinh; A4 bo gio; A11 bo trang thai dien) gom:
%     afp   rung hoi quy luong tu [R39] tau = 0.9 cua m (qrf_predict) + CQR mot phia [R32],
%           hieu chuan toan cuc va theo nhom muc gio (cqr_calibrate)
%     prog, ppk   toc do tien do, cong suat dinh (chi phu thuoc quy dao, khong f_c)
%     h2, eb, dg  H2, cong suat pin + sieu tu, suy giam chuan hoa (phu thuoc ca f_c)
%   Sieu tham so rung (GT): 60 cay, la toi thieu 20 mau, so dac trung moi lan chia = 1/3
%   ([R39] dung 1000 cay, la > 10, mtry = p/3; ket qua it nhay voi mtry va co la).
%   rep.test: do phu, pinball, recall/bao dong gia, AUC cua AFP va R^2 tren tap kiem tra.
if nargin < 2, opts = struct(); end
def = struct('ntrees', 60, 'leaf', 20, 'tau', 0.9, 'alpha', 0.1);
f = fieldnames(def);
for k = 1:numel(f), if ~isfield(opts, f{k}), opts.(f{k}) = def.(f{k}); end, end
if ~isfield(opts, 'variants'), opts.variants = {'full', 'A3', 'A4', 'A11'}; end
G = D.groups;
sp = e3_split(D.flight);
tr = sp.tr; ca = sp.ca; te = sp.te; ok = ~D.crash;

nm = struct('H', 10, 'V_ref', 25, 'LHV', 120e6, 's0', 2.5, 'k_soc', 3, 'soc_floor', 0.35);
nm.h2_ref = median(D.h2(tr & ok));
nm.med_bd = median(D.bd(tr & ok)); nm.med_fd = median(D.fd(tr & ok));
dg = 0.5*(D.bd/nm.med_bd + D.fd/nm.med_fd);
M.norm = nm; M.split = sp; M.names = D.names; M.opts = opts;

drop = struct('full', [], 'A3', G.config, 'A4', G.wind, 'A11', G.elec);
rows = {};
for v = opts.variants
    cols = setdiff(1:size(D.X, 2), drop.(v{1}));
    colsT = setdiff(cols, G.cand(4));
    fprintf('[e3_train] %s: %d dac trung, %d mau huan luyen\n', v{1}, numel(cols), nnz(tr));
    mdl.tau = opts.tau;
    mdl.afp = fit_forest(D.X(tr, colsT), D.m(tr), opts, true); mdl.afp.cols = colsT;
    qc = qrf_predict(mdl.afp, D.X(ca, colsT), opts.tau);
    mdl.C = cqr_calibrate(qc, D.m(ca), opts.alpha, D.wbin(ca));
    r = tr & ok;
    mdl.prog = fit_forest(D.X(r, colsT), D.prog(r), opts, false); mdl.prog.cols = colsT;
    mdl.ppk  = fit_forest(D.X(r, colsT), D.ppk(r), opts, false);  mdl.ppk.cols = colsT;
    mdl.h2   = fit_forest(D.X(r, cols), D.h2(r), opts, false);    mdl.h2.cols = cols;
    mdl.eb   = fit_forest(D.X(r, cols), D.eb(r), opts, false);    mdl.eb.cols = cols;
    mdl.dg   = fit_forest(D.X(r, cols), dg(r), opts, false);      mdl.dg.cols = cols;
    M.(v{1}) = mdl;

    % Danh gia tren tap kiem tra
    qt = qrf_predict(mdl.afp, D.X(te, colsT), opts.tau);
    row = afp_metrics(qt, D.m(te), mdl.C, D.wbin(te), opts.tau);
    rt = te & ok;
    row.R2_prog = r2(rf_predict(mdl.prog, D.X(rt, colsT)), D.prog(rt));
    row.R2_ppk  = r2(rf_predict(mdl.ppk, D.X(rt, colsT)), D.ppk(rt));
    row.R2_h2   = r2(rf_predict(mdl.h2, D.X(rt, cols)), D.h2(rt));
    row.R2_eb   = r2(rf_predict(mdl.eb, D.X(rt, cols)), D.eb(rt));
    row.R2_dg   = r2(rf_predict(mdl.dg, D.X(rt, cols)), dg(rt));
    row.variant = string(v{1}); row.n_train = nnz(tr);
    rows{end+1} = row; %#ok<AGROW>
end
rep.test = struct2table([rows{:}]');
rep.test = movevars(rep.test, {'variant', 'n_train'}, 'Before', 1);
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
