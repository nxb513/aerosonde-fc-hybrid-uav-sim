function LC = learning_curve(T1, T2, outdir, opts)
%LEARNING_CURVE Duong hoc cua AFP theo luong du lieu E1, co checkpoint mo hinh.
%   LC = LEARNING_CURVE(T1, T2, outdir, opts)
%   Chia E1 theo thu tu sinh (in_id, cung thu tu checkpoint luc thu thap):
%     60 % dau = kho huan luyen, 20 % tiep = hieu chuan (co dinh), 20 % cuoi = test.
%   Tai moi moc n (opts.sizes): huan luyen AFP tren n mau dau cua kho huan luyen,
%   hieu chuan CQR [R32] tren tap hieu chuan co dinh, luu checkpoint mo hinh
%   (outdir/ckpt_afp_nXXXXX.mat), danh gia tren E1-test va mot mau con E2:
%     pinball loss (train/test), do phu, do du, recall/bao dong gia vi pham, AUC.
%   Them opts.repeats lan lay mau ngau nhien tu kho huan luyen de ve dai bien thien.
%   Cuoi cung: sai so out-of-bag theo so cay cua mo hinh day du (duong hoc theo cay).
if nargin < 4, opts = struct(); end
if ~isfield(opts, 'sizes'), opts.sizes = [250 500 1000 2000 3000 4000 6000]; end
if ~isfield(opts, 'repeats'), opts.repeats = 3; end
if ~isfield(opts, 'alpha'), opts.alpha = 0.1; end
if ~isfield(opts, 'n_e2'), opts.n_e2 = 5000; end
if ~exist(outdir, 'dir'), mkdir(outdir); end
tau = 1 - opts.alpha;

[~, o] = sort(T1.in_id); T1 = T1(o, :);
D1 = afp_dataset(T1); D2 = afp_dataset(T2);
n = size(D1.X, 1);
itr = 1:round(0.6*n); ical = round(0.6*n)+1:round(0.8*n); ite = round(0.8*n)+1:n;
rs = RandStream('mt19937ar', 'Seed', 3);
i2 = randperm(rs, size(D2.X, 1), min(opts.n_e2, size(D2.X, 1)));
sizes = opts.sizes(opts.sizes <= numel(itr));
if sizes(end) < numel(itr), sizes(end+1) = numel(itr); end

rows = {};
for r = 0:opts.repeats
    if r == 0
        pool = itr;                       % thu tu sinh = thu tu checkpoint
    else
        pool = itr(randperm(rs, numel(itr)));
    end
    for s = sizes
        sel = pool(1:s);
        M = afp_fit(D1.X(sel, :), D1.m(sel), tau);
        C = cqr_calibrate(afp_predict(M, D1.X(ical, :)), D1.m(ical), opts.alpha);
        if r == 0
            Mc = M; Mc.B = compact(M.B); %#ok<NASGU>
            save(fullfile(outdir, sprintf('ckpt_afp_n%05d.mat', s)), 'Mc', 'C', 'sel', '-v7');
        end
        q_tr = afp_predict(M, D1.X(sel, :));
        row = struct('order', r, 'n', s, 'pinball_train', pinball(D1.m(sel), q_tr, tau));
        for tst = {'E1', 'E2'}
            if strcmp(tst{1}, 'E1'), X = D1.X(ite, :); y = D1.m(ite);
            else, X = D2.X(i2, :); y = D2.m(i2); end
            q = afp_predict(M, X); ub = cqr_bound(C, q);
            tv = y > 1; pv = ub > 1;
            row.(['pinball_' tst{1}]) = pinball(y, q, tau);
            row.(['coverage_' tst{1}]) = mean(y <= ub);
            row.(['excess_' tst{1}]) = mean(max(ub - y, 0));
            row.(['recall_' tst{1}]) = sum(pv & tv)/max(sum(tv), 1);
            row.(['falsealarm_' tst{1}]) = sum(pv & ~tv)/max(sum(~tv), 1);
            row.(['auc_' tst{1}]) = auc(ub, tv);
        end
        rows{end+1} = row; %#ok<AGROW>
        fprintf('learning curve: lan %d, n = %d, pinball E1 = %.4f, phu E1 = %.3f\n', ...
            r, s, row.pinball_E1, row.coverage_E1);
    end
end
LC.table = struct2table([rows{:}]');
writetable(LC.table, fullfile(outdir, 'learning_curve.csv'));

% Duong hoc theo so cay (out-of-bag) cho mo hinh day du
rng(42);
Bo = TreeBagger(200, D1.X(itr, :), D1.m(itr), 'Method', 'regression', 'MinLeafSize', 5, ...
    'NumPredictorsToSample', max(1, round(size(D1.X, 2)/3)), 'OOBPrediction', 'on');
LC.oob_mse = oobError(Bo);
writematrix([(1:numel(LC.oob_mse))', LC.oob_mse(:)], fullfile(outdir, 'oob_vs_trees.csv'));
plot_lc(LC, sizes, opts, outdir);
end

function L = pinball(y, q, tau)
e = y - q;
L = mean(max(tau*e, (tau - 1)*e));
end

function a = auc(score, lab)
% AUC theo thong ke Mann-Whitney (khong can toolbox)
pos = score(lab); neg = score(~lab);
if isempty(pos) || isempty(neg), a = NaN; return; end
r = tiedrank_simple([pos; neg]);
a = (sum(r(1:numel(pos))) - numel(pos)*(numel(pos) + 1)/2)/(numel(pos)*numel(neg));
end

function r = tiedrank_simple(x)
[xs, o] = sort(x); r = zeros(size(x)); n = numel(x); k = 1;
while k <= n
    j = k; while j < n && xs(j + 1) == xs(k), j = j + 1; end
    r(o(k:j)) = (k + j)/2; k = j + 1;
end
end

function plot_lc(LC, sizes, opts, outdir)
T = LC.table; m = T(T.order == 0, :); R = T(T.order > 0, :);
f = figure('Visible', 'off', 'Position', [100 100 1150 760]);
band = @(v) [arrayfun(@(s) min(R.(v)(R.n == s)), sizes); arrayfun(@(s) max(R.(v)(R.n == s)), sizes)];
subplot(2, 2, 1);
semilogx(m.n, m.pinball_train, '-o', m.n, m.pinball_E1, '-s', m.n, m.pinball_E2, '-d', 'LineWidth', 1.2); hold on;
b = band('pinball_E1'); fill([sizes fliplr(sizes)], [b(1, :) fliplr(b(2, :))], [0.85 0.33 0.1], 'FaceAlpha', 0.15, 'EdgeColor', 'none');
grid on; xlabel('So mau huan luyen E1 (checkpoint)'); ylabel(sprintf('Pinball loss (\\tau = %.2f)', 1 - opts.alpha));
legend('train', 'E1 test', 'E2', 'Location', 'northeast'); title('Duong hoc AFP');
subplot(2, 2, 2);
semilogx(m.n, m.coverage_E1, '-s', m.n, m.coverage_E2, '-d', 'LineWidth', 1.2); hold on;
yline(1 - opts.alpha, '--k', '1 - \alpha'); grid on; ylim([0.7 1]);
xlabel('So mau huan luyen E1'); ylabel('Do phu sau CQR'); legend('E1 test', 'E2', 'Location', 'southeast');
subplot(2, 2, 3);
semilogx(m.n, m.recall_E1, '-s', m.n, m.falsealarm_E1, '-o', m.n, m.auc_E1, '-^', 'LineWidth', 1.2); grid on;
xlabel('So mau huan luyen E1'); ylabel('E1 test'); legend('recall vi pham', 'bao dong gia', 'AUC', 'Location', 'best');
subplot(2, 2, 4);
plot(LC.oob_mse, 'LineWidth', 1.2); grid on; xlabel('So cay'); ylabel('MSE out-of-bag');
title('Duong hoc theo so cay (mo hinh day du)');
exportgraphics(f, fullfile(outdir, 'learning_curve.png'), 'Resolution', 130); close(f);
end
