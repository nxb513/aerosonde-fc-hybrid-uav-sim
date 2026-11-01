function out = run_analysis(dir1, dir2, outdir)
%RUN_ANALYSIS Chuoi phan tich day du: AFP + conformal, quyet dinh, ablation, nguon.
%   out = RUN_ANALYSIS(dir1, dir2, outdir)
%     dir1, dir2  thu muc chua all_E1.mat / all_E2.mat (artifact da gop) hoac chunk_*.mat
%   Ket qua: cac bang CSV + hinh + summary.md trong outdir.
root = fileparts(fileparts(mfilename('fullpath')));
if nargin < 1, dir1 = fullfile(root, 'results', 'E1'); end
if nargin < 2, dir2 = fullfile(root, 'results', 'E2'); end
if nargin < 3, outdir = fullfile(root, 'results', 'analysis'); end
[T1, ~] = load_exp('E1', dir1);
[T2, tr2] = load_exp('E2', dir2);
out = analysis_core(T1, T2, tr2, outdir);
end

function [T, tr] = load_exp(exp, d)
f = fullfile(d, sprintf('all_%s.mat', exp));
if exist(f, 'file')
    L = load(f); T = L.T; tr = L.traces;
else
    [T, tr] = gather_results(exp, d);
end
end

function out = analysis_core(T1, T2, tr2, outdir)
if ~exist(outdir, 'dir'), mkdir(outdir); end
fprintf('E1: %d, E2: %d kich ban\n', height(T1), height(T2));

%% 0. Duong hoc AFP theo luong du lieu E1 (checkpoint mo hinh trong outdir/learning)
out.lc = learning_curve(T1, T2, fullfile(outdir, 'learning'));

%% 1. AFP + conformal (va ablation A2, A3, A4, A10)
out.afp = table();
for alpha = [0.1 0.05]
    [R, models] = afp_ablation(T1, T2, alpha);
    R.alpha = repmat(alpha, height(R), 1);
    out.afp = [out.afp; R];
    if alpha == 0.1, M10 = models; end
end
writetable(out.afp, fullfile(outdir, 'afp_ablation.csv'));

%% 2. Quyet dinh: oracle / ours / ablation A1-A8 / baseline
S = surrogate_fit(T1);
[out.dec, choice] = decision_eval(T2, M10, S, [0.2 0.5 0.8]);
writetable(out.dec, fullfile(outdir, 'decision.csv'));

%% 3. Ablation phia nguon tren quy dao do chinh sach chon (lambda = 0.5)
ch = choice.L05;
out.pow = table();
for pol = {'ours', 'A7_comp_only', 'A6_replan_only'}
    rows = unique(ch.(pol{1}));
    rows = rows(cellfun(@(t) ~isempty(t), tr2(rows)));
    R = power_ablation(T2, tr2, rows, 1/20);
    R.policy = repmat(string(pol{1}), height(R), 1);
    out.pow = [out.pow; R];
end
writetable(out.pow, fullfile(outdir, 'power_ablation.csv'));

%% 4. Mo ta vat ly tu E2: ty le vi pham theo CG, gio, bank
D2 = afp_dataset(T2);
G = groupsummary(table(T2.in_dx_cg, T2.in_W_h, T2.in_phi_max, double(D2.viol), ...
    'VariableNames', {'dx_cg','W_h','phi_max','viol'}), {'dx_cg','W_h','phi_max'}, 'mean', 'viol');
writetable(G, fullfile(outdir, 'viol_by_cg_wind_bank.csv'));
out.viol_map = G;

%% 5. So mau Monte Carlo (cận Tempo [R21])
eps_ = 0.01; del = 0.01;
out.tempo = struct('chernoff', ceil(log(2/del)/(2*eps_^2)), ...
                   'worst_case', ceil(log(1/del)/log(1/(1 - eps_))), ...
                   'n_E1', height(T1), 'n_E2', height(T2));

plot_results(out, outdir);
write_summary(out, outdir);
save(fullfile(outdir, 'analysis.mat'), 'out');
end

function plot_results(out, outdir)
f = figure('Visible', 'off', 'Position', [100 100 1100 380]);
G = out.viol_map;
cg = unique(G.dx_cg);
for i = 1:numel(cg)
    subplot(1, numel(cg), i);
    g = G(G.dx_cg == cg(i), :);
    W = unique(g.W_h); P = unique(g.phi_max);
    Z = nan(numel(W), numel(P));
    for a = 1:numel(W), for b = 1:numel(P)
        Z(a, b) = g.mean_viol(g.W_h == W(a) & g.phi_max == P(b));
    end, end
    imagesc(P, W, Z, [0 1]); axis xy; colorbar;
    xlabel('\phi_{max} (deg)'); ylabel('W_h (m/s)');
    title(sprintf('Ty le vi pham, \\Deltax_{cg} = %+.1f cm', 100*cg(i)));
end
exportgraphics(f, fullfile(outdir, 'viol_map.png'), 'Resolution', 130); close(f);

f = figure('Visible', 'off', 'Position', [100 100 900 380]);
A = out.afp(out.afp.alpha == 0.1, :);
v = unique(A.variant, 'stable');
c1 = arrayfun(@(x) A.coverage(A.variant == x & A.test == "E1_test"), v);
c2 = arrayfun(@(x) A.coverage(A.variant == x & A.test == "E2"), v);
bar(categorical(v, v), [c1 c2]); hold on; yline(0.9, '--k', '1 - \alpha');
ylim([0.5 1]); legend('E1 test', 'E2', 'Location', 'southwest'); ylabel('Do phu');
title('AFP: do phu can tren (alpha = 0.1)');
exportgraphics(f, fullfile(outdir, 'afp_coverage.png'), 'Resolution', 130); close(f);
end

function write_summary(out, outdir)
fid = fopen(fullfile(outdir, 'summary.md'), 'w');
fprintf(fid, '# Tom tat phan tich\n\nE1 = %d kich ban, E2 = %d kich ban.\n\n', out.tempo.n_E1, out.tempo.n_E2);
fprintf(fid, 'Can Tempo [R21] (eps = delta = 0.01): Chernoff N >= %d, truong hop xau nhat N >= %d.\n\n', ...
    out.tempo.chernoff, out.tempo.worst_case);
fprintf(fid, '## AFP (alpha = 0.1)\n\n'); write_tab(fid, out.afp(out.afp.alpha == 0.1, :));
fprintf(fid, '\n## Quyet dinh (lambda = 0.5)\n\n'); write_tab(fid, out.dec(out.dec.lambda == 0.5, :));
fprintf(fid, '\n## Ablation nguon\n\n'); write_tab(fid, out.pow);
fclose(fid);
end

function write_tab(fid, T)
v = T.Properties.VariableNames;
fprintf(fid, '| %s |\n|%s\n', strjoin(v, ' | '), repmat('---|', 1, numel(v)));
for i = 1:height(T)
    c = cell(1, numel(v));
    for j = 1:numel(v)
        x = T{i, j};
        if isstring(x) || ischar(x), c{j} = char(x); else, c{j} = sprintf('%.4g', x); end
    end
    fprintf(fid, '| %s |\n', strjoin(c, ' | '));
end
end
