function [R, models] = afp_ablation(T1, T2, alpha)
%AFP_ABLATION Huan luyen/hieu chuan AFP va cac bien the ablation, danh gia do phu.
%   [R, models] = AFP_ABLATION(T1, T2, alpha)
%     T1  ket qua E1 (chia 60/20/20: train/cal/test, trao doi duoc)
%     T2  ket qua E2 (luoi co cau truc: kiem tra khi phan phoi lech)
%   Bien the: full | A2 khong conformal | A3 khong cau hinh+vat ly | A4 khong gio
%             | khong dac trung vat ly (trim) | A10 conformal theo nhom gio.
%   Chi so: do phu P(m <= can tren), do phu nho nhat theo nhom gio, recall /
%   ty le bao dong gia khi phat hien vi pham (can tren > 1), do du trung binh.
if nargin < 3, alpha = 0.1; end
D1 = afp_dataset(T1); D2 = afp_dataset(T2);
n = size(D1.X, 1);
rs = RandStream('mt19937ar', 'Seed', 1);
pidx = randperm(rs, n);
itr = pidx(1:round(0.6*n)); ical = pidx(round(0.6*n)+1:round(0.8*n)); ite = pidx(round(0.8*n)+1:end);
G = D1.groups;
all_cols = 1:size(D1.X, 2);
V = {
  'full',          all_cols,                                   'global'
  'A10_group',     all_cols,                                   'group'
  'A2_no_conf',    all_cols,                                   'none'
  'A3_no_config',  setdiff(all_cols, [G.config G.phys]),       'global'
  'A4_no_wind',    setdiff(all_cols, G.wind),                  'global'
  'no_phys',       setdiff(all_cols, G.phys),                  'global'
};
R = table(); models = struct();
for v = 1:size(V, 1)
    M = afp_fit(D1.X(itr, :), D1.m(itr), 1 - alpha, struct('cols', V{v, 2}));
    qc = afp_predict(M, D1.X(ical, :));
    C = cqr_calibrate(qc, D1.m(ical), alpha, D1.wind_bin(ical));
    mode = V{v, 3};
    for test = {'E1_test', 'E2'}
        if strcmp(test{1}, 'E1_test')
            X = D1.X(ite, :); y = D1.m(ite); g = D1.wind_bin(ite);
        else
            X = D2.X; y = D2.m; g = D2.wind_bin;
        end
        q = afp_predict(M, X);
        switch mode
            case 'none',   ub = q;
            case 'global', ub = cqr_bound(C, q);
            case 'group',  ub = cqr_bound(C, q, g);
        end
        cov = mean(y <= ub);
        covg = arrayfun(@(k) mean(y(g == k) <= ub(g == k)), unique(g)');
        tv = y > 1; pv = ub > 1;
        row = table(string(V{v, 1}), string(test{1}), numel(y), cov, min(covg), ...
            sum(pv & tv)/max(sum(tv), 1), sum(pv & ~tv)/max(sum(~tv), 1), ...
            mean(max(ub - y, 0)), mean(tv), ...
            'VariableNames', {'variant','test','n','coverage','coverage_min_group', ...
            'recall_viol','false_alarm','mean_excess','viol_rate'});
        R = [R; row]; %#ok<AGROW>
    end
    models.(V{v, 1}) = struct('M', M, 'C', C, 'mode', mode);
end
models.alpha = alpha;
end
