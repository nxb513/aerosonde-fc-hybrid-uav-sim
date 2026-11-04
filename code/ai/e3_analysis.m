function R = e3_analysis(indir, exp)
%E3_ANALYSIS Phan tich thi nghiem danh gia E3 (so sanh chinh sach, ablation, do nhay).
%   R = E3_ANALYSIS(indir, exp); mac dinh indir = results/<exp>, exp = 'E3'
%   (all_<exp>.mat tu gather_results).
%   Moi chinh sach va ban do nhay bay cung 48 kich ban (cung chuoi nhiem vu, gio, khoi
%   dau): so sanh theo cap voi 'ours' bang kiem dinh hang co dau Wilcoxon (signrank).
%   Chi so moi chuyen bay (bao_cao/thiet_ke_E3.pdf muc 8):
%     n_ok        so nhiem vu dat (bay xong, m <= 1) (chinh)
%     crash, n_viol, viol_time, thoi gian cham tung gioi han cung (lim_*)
%     H2_per_ok   H2 tuong duong (hieu chinh SoC, sieu tu cuoi chuyen [R30]) / nhiem vu dat (g)
%     t_per_ok    thoi gian trung binh moi nhiem vu dat (s); u2_mean muc dung canh lai
%     usd_per_ok  chi phi van hanh USD / nhiem vu dat (gia [R14], suy giam [R16][R17][R18])
%   Do phu thuc te tren quy dao cua chinh sach: cua so 10 s sau moi quyet dinh,
%   P(m <= u) (R1) va P(alpha_p99/alpha0 <= a) (R2), toan bo va theo nhom gio.
%   Bao cao (indir/report): policy_summary.csv, paired_vs_ours.csv, by_wind.csv,
%   by_config.csv, task_ok_by_type.csv, onpolicy_coverage.csv, summary.md, policies.png.
root = fileparts(fileparts(mfilename('fullpath')));
if nargin < 2, exp = 'E3'; end
if nargin < 1 || isempty(indir), indir = fullfile(root, 'results', exp); end
out = fullfile(indir, 'report'); if ~exist(out, 'dir'), mkdir(out); end
S = load(fullfile(indir, sprintf('all_%s.mat', exp)));
T = S.T; tr = S.traces;
T.crash = double(T.stop_reason == "crash");
T.ok = T.ok == 1;
lims = T.Properties.VariableNames(startsWith(T.Properties.VariableNames, 'lim_'));
mets = [{'n_ok', 'crash', 'n_viol', 'viol_time', 'H2_per_ok', 't_per_ok', 'u2_mean', ...
         'usd_per_ok', 'usd', 't_flight', 'soc_end', 'fc_uV', 'bat_dmg', 'n_done'}, lims];
better_high = {'n_ok', 'n_done'};
pols = unique(T.in_policy, 'stable');

% ---- Tong hop theo chinh sach
rows = {};
for i = 1:numel(pols)
    t = T(T.in_policy == pols(i), :);
    r = struct('policy', pols(i), 'n', height(t), 'fail', mean(~t.ok), ...
               'stop_h2', mean(t.stop_reason == "h2"), 'stop_soc', mean(t.stop_reason == "soc"), ...
               'stop_tmax', mean(t.stop_reason == "tmax"));
    for m = mets
        x = t.(m{1}); r.([m{1} '_mean']) = mean(x, 'omitnan'); r.([m{1} '_med']) = median(x, 'omitnan');
    end
    if ismember('n_lvl0', t.Properties.VariableNames)
        nd = sum(t.n_dec, 'omitnan');
        r.frac_lvl1 = sum(t.n_lvl1, 'omitnan')/max(nd, 1); r.frac_lvl2 = sum(t.n_lvl2, 'omitnan')/max(nd, 1);
    end
    rows{end+1} = r; %#ok<AGROW>
end
R.summary = struct2table([rows{:}]');
writetable(R.summary, fullfile(out, 'policy_summary.csv'));

% ---- So sanh cap voi 'ours'
ref = T(T.in_policy == "ours", :);
rows = {};
for i = 1:numel(pols)
    if pols(i) == "ours", continue; end
    t = T(T.in_policy == pols(i), :);
    [~, ia, ib] = intersect(t.in_seed, ref.in_seed);
    for m = mets
        a = t.(m{1})(ia); b = ref.(m{1})(ib);
        g = isfinite(a) & isfinite(b);
        d = a(g) - b(g);
        r = struct('policy', pols(i), 'metric', string(m{1}), 'n_pairs', nnz(g), ...
                   'mean_diff', mean(d), 'median_diff', median(d), 'p_signrank', NaN, ...
                   'ours_better', NaN);
        if nnz(g) >= 5 && any(d ~= 0), r.p_signrank = signrank(d); end
        if ismember(m{1}, better_high), r.ours_better = mean(d < 0); else, r.ours_better = mean(d > 0); end
        rows{end+1} = r; %#ok<AGROW>
    end
end
R.paired = struct2table([rows{:}]');
writetable(R.paired, fullfile(out, 'paired_vs_ours.csv'));

% ---- Theo muc gio va cau hinh
R.by_wind = groupsummary(T, {'in_policy', 'in_W_lvl'}, {'mean'}, {'n_ok', 'crash', 'n_viol', 'H2_per_ok'});
writetable(R.by_wind, fullfile(out, 'by_wind.csv'));
R.by_config = groupsummary(T, {'in_policy', 'in_dx_cg'}, {'mean'}, {'n_ok', 'crash', 'n_viol', 'H2_per_ok'});
writetable(R.by_config, fullfile(out, 'by_config.csv'));

% ---- Ti le nhiem vu dat theo loai
names = {'transit', 'turn', 'climb', 'loiter', 'survey', 'gust_hold'};
rows = {};
for i = 1:numel(pols)
    TK = [];
    for j = find(T.in_policy == pols(i)).'
        if isstruct(tr{j}) && isfield(tr{j}, 'tasks'), TK = [TK; double(tr{j}.tasks)]; end %#ok<AGROW>
    end
    if isempty(TK), continue; end
    TK = TK(TK(:, end) == 0, :);                      % bo nhiem vu bi cat do dung chuyen bay
    r = struct('policy', pols(i));
    for k = 1:6
        q = TK(:, 2) == k;
        r.([names{k} '_ok']) = mean(TK(q, 7)); r.([names{k} '_viol']) = mean(TK(q, 8) > 1);
    end
    rows{end+1} = r; %#ok<AGROW>
end
R.task = struct2table([rows{:}]');
writetable(R.task, fullfile(out, 'task_ok_by_type.csv'));

% ---- Do phu thuc te tren quy dao cua chinh sach (cua so du 10 s)
rows = {};
for i = 1:numel(pols)
    O = [];
    for j = find(T.in_policy == pols(i)).'
        if isstruct(tr{j}) && isfield(tr{j}, 'onpolicy') && ~isempty(tr{j}.onpolicy)
            O = [O; double(tr{j}.onpolicy)]; nmo = tr{j}.onpolicy_names; %#ok<AGROW>
        end
    end
    if isempty(O), continue; end
    c = @(n) O(:, strcmp(nmo, n));
    full = c('dur') >= 10 - 1e-6 & c('crash') == 0;
    m = c('m'); u = c('u'); a = c('a_real'); ab = c('a'); g = c('grp');
    r = struct('policy', pols(i), 'n', nnz(full), ...
               'cov_R1', mean(m(full) <= u(full)), 'cov_R2', mean(a(full) <= ab(full)), ...
               'viol', mean(m(full) > 1), 'viol_when_u_le1', mean(m(full & u <= 1) > 1), ...
               'stall_exceed', mean(a(full) > 1));
    for k = 1:4
        s = full & g == k;
        r.(sprintf('cov_R1_g%d', k)) = mean(m(s) <= u(s));
        r.(sprintf('cov_R2_g%d', k)) = mean(a(s) <= ab(s));
    end
    rows{end+1} = r; %#ok<AGROW>
end
if isempty(rows), R.onpolicy = table(); else, R.onpolicy = struct2table([rows{:}]'); end
if ~isempty(R.onpolicy), writetable(R.onpolicy, fullfile(out, 'onpolicy_coverage.csv')); end

% ---- Hinh
fig = figure('Visible', 'off', 'Position', [100 100 1100 520]);
s = R.summary; [~, o] = sort(s.n_ok_mean, 'descend'); s = s(o, :);
subplot(1, 2, 1); barh(categorical(s.policy, s.policy), s.n_ok_mean); xlabel('so nhiem vu dat (trung binh)'); grid on
subplot(1, 2, 2); barh(categorical(s.policy, s.policy), s.crash_mean); xlabel('ti le roi may'); grid on
exportgraphics(fig, fullfile(out, 'policies.png'), 'Resolution', 150);
close(fig);

% ---- Tom tat
fid = fopen(fullfile(out, 'summary.md'), 'w');
fprintf(fid, '# E3: so sanh chinh sach (bay den khi het H2, SoC < 30 %% hoac roi)\n\n');
fprintf(fid, '%d chuyen bay; %d chinh sach/bien the.\n\n', height(T), numel(pols));
fprintf(fid, ['| chinh sach | n | loi | roi | n_ok TB | n_ok TV | n_viol TB | H2/nv dat (g) | ' ...
              't/nv dat (s) | u2 TB | USD/nv dat | dung tmax |\n|---|---|---|---|---|---|---|---|---|---|---|---|\n']);
for i = 1:height(s)
    fprintf(fid, '| %s | %d | %.3f | %.3f | %.2f | %.1f | %.2f | %.3f | %.0f | %.3f | %.4f | %.3f |\n', ...
        s.policy(i), s.n(i), s.fail(i), s.crash_mean(i), s.n_ok_mean(i), s.n_ok_med(i), ...
        s.n_viol_mean(i), s.H2_per_ok_mean(i), s.t_per_ok_mean(i), s.u2_mean_mean(i), ...
        s.usd_per_ok_mean(i), s.stop_tmax(i));
end
fprintf(fid, '\n## So sanh cap voi ours (n_ok, crash, H2_per_ok)\n\n| chinh sach | chi so | cap | TB(chinh sach - ours) | p Wilcoxon | ti le ours tot hon |\n|---|---|---|---|---|---|\n');
P = R.paired(ismember(R.paired.metric, ["n_ok", "crash", "H2_per_ok"]), :);
for i = 1:height(P)
    fprintf(fid, '| %s | %s | %d | %.3f | %.2g | %.2f |\n', P.policy(i), P.metric(i), P.n_pairs(i), ...
        P.mean_diff(i), P.p_signrank(i), P.ours_better(i));
end
if ~isempty(R.onpolicy)
    O = R.onpolicy;
    fprintf(fid, '\n## Do phu thuc te tren quy dao cua chinh sach (cua so 10 s)\n\n| chinh sach | n | phu R1 | phu R2 | vi pham | vi pham khi u<=1 | vuot stall |\n|---|---|---|---|---|---|---|\n');
    for i = 1:height(O)
        fprintf(fid, '| %s | %d | %.3f | %.4f | %.3f | %.3f | %.4f |\n', O.policy(i), O.n(i), ...
            O.cov_R1(i), O.cov_R2(i), O.viol(i), O.viol_when_u_le1(i), O.stall_exceed(i));
    end
end
fclose(fid);
fprintf('%s: phan tich xong, bao cao trong %s\n', exp, out);
end
