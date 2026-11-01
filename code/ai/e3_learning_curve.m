function LC = e3_learning_curve(D, sp, outdir, opts)
%E3_LEARNING_CURVE Duong hoc cua AFP theo luong du lieu tham do E3X.
%   LC = E3_LEARNING_CURVE(D, sp, outdir, opts)
%   Tap hieu chuan va tap kiem tra co dinh (sp = e3_split). Tap huan luyen tang dan theo
%   so chuyen bay tham do, lay theo thu tu id (thu tu thu du lieu); them 3 lan lay tap
%   con ngau nhien cung kich thuoc de co dai bien thien. Moi moc luu checkpoint mo hinh
%   (afp dong goi + hieu chuan) vao outdir/ckpt/afp_fXXXX.mat.
%   Chi so: pinball (huan luyen, kiem tra), do phu toan cuc/theo nhom, recall, bao dong
%   gia, AUC (afp_metrics). Them loi OOB theo so cay tren toan bo tap huan luyen.
%   Ket qua: learning_curve.csv, oob_vs_trees.csv, learning_curve.png.
if nargin < 4, opts = struct(); end
def = struct('ntrees', 60, 'leaf', 20, 'tau', 0.9, 'alpha', 0.1, 'reps', 3, ...
             'sizes', [25 50 100 200 400 800]);
f = fieldnames(def);
for k = 1:numel(f), if ~isfield(opts, f{k}), opts.(f{k}) = def.(f{k}); end, end
ck = fullfile(outdir, 'ckpt'); if ~exist(ck, 'dir'), mkdir(ck); end
cols = setdiff(1:size(D.X, 2), D.groups.cand(4));
ftr = sp.f_tr(:);
sizes = unique([opts.sizes(opts.sizes < numel(ftr)), numel(ftr)]);
rs = RandStream('mt19937ar', 'Seed', 5);
rows = {};
for n = sizes
    for rep = 0:opts.reps
        if rep == 0
            fsel = ftr(1:n);                         % theo thu tu thu du lieu
        else
            if n == numel(ftr), continue; end
            fsel = ftr(sort(randperm(rs, numel(ftr), n)));
        end
        r = ismember(D.flight, fsel);
        rng(42);
        B = TreeBagger(opts.ntrees, D.X(r, cols), D.m(r), 'Method', 'regression', ...
            'MinLeafSize', opts.leaf, 'NumPredictorsToSample', max(1, round(numel(cols)/3)));
        P = rf_pack(B, D.X(r, cols), D.m(r)); P.cols = cols;
        C = cqr_calibrate(qrf_predict(P, D.X(sp.ca, cols), opts.tau), D.m(sp.ca), opts.alpha, D.wbin(sp.ca));
        row = afp_metrics(qrf_predict(P, D.X(sp.te, cols), opts.tau), D.m(sp.te), C, D.wbin(sp.te), opts.tau);
        qtr = qrf_predict(P, D.X(r, cols), opts.tau);
        row.pinball_train = mean(max(opts.tau*(D.m(r) - qtr), (opts.tau - 1)*(D.m(r) - qtr)));
        row.n_flights = n; row.n_rows = nnz(r); row.rep = rep;
        rows{end+1} = row; %#ok<AGROW>
        if rep == 0
            afp = P; tau = opts.tau; %#ok<NASGU>
            save(fullfile(ck, sprintf('afp_f%04d.mat', n)), 'afp', 'C', 'tau', 'fsel', '-v7');
        end
        fprintf('[learning curve] %d chuyen bay (lan %d): pinball %.4f, phu %.3f, AUC %.3f\n', ...
            n, rep, row.pinball, row.cov_group, row.auc);
    end
end
LC = struct2table([rows{:}]');
LC = movevars(LC, {'n_flights', 'n_rows', 'rep'}, 'Before', 1);
writetable(LC, fullfile(outdir, 'learning_curve.csv'));

% Loi OOB theo so cay (tren toan bo tap huan luyen)
rng(42);
B = TreeBagger(150, D.X(sp.tr, cols), D.m(sp.tr), 'Method', 'regression', 'MinLeafSize', opts.leaf, ...
    'NumPredictorsToSample', max(1, round(numel(cols)/3)), 'OOBPrediction', 'on');
oob = oobError(B);
writetable(table((1:numel(oob)).', oob(:), 'VariableNames', {'n_trees', 'oob_mse'}), ...
    fullfile(outdir, 'oob_vs_trees.csv'));

% Hinh
fig = figure('Visible', 'off', 'Position', [100 100 1100 360]);
o = LC.rep == 0; z = LC.rep > 0;
subplot(1, 3, 1);
semilogx(LC.n_flights(o), LC.pinball(o), 'o-', LC.n_flights(o), LC.pinball_train(o), 's--'); hold on
semilogx(LC.n_flights(z), LC.pinball(z), '.', 'Color', [0.5 0.5 0.5]);
xlabel('so chuyen bay huan luyen'); ylabel('pinball (\tau = 0.9)'); legend('kiem tra', 'huan luyen', 'tap con ngau nhien');
grid on
subplot(1, 3, 2);
semilogx(LC.n_flights(o), LC.cov_group(o), 'o-', LC.n_flights(o), LC.cov_global(o), 's--');
yline(1 - opts.alpha, ':'); xlabel('so chuyen bay huan luyen'); ylabel('do phu');
legend('theo nhom gio', 'toan cuc', '1 - \alpha'); grid on
subplot(1, 3, 3);
semilogx(LC.n_flights(o), LC.auc(o), 'o-', LC.n_flights(o), LC.recall(o), 's-', ...
    LC.n_flights(o), LC.false_alarm(o), 'd-');
xlabel('so chuyen bay huan luyen'); legend('AUC', 'recall', 'bao dong gia'); grid on
exportgraphics(fig, fullfile(outdir, 'learning_curve.png'), 'Resolution', 150);
close(fig);
end
