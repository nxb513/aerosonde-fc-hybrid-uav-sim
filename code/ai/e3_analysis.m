function R = e3_analysis(indir)
%E3_ANALYSIS Phan tich thi nghiem danh gia E3 (so sanh chinh sach, ablation, do nhay).
%   R = E3_ANALYSIS(indir); mac dinh indir = results/E3 (all_E3.mat tu gather_results).
%   Moi chinh sach bay cung 144 kich ban (cung chuoi nhiem vu, gio, khoi dau): so sanh
%   theo cap voi 'ours' bang kiem dinh hang co dau Wilcoxon (signrank) tren cac kich ban
%   chung; S1/S2 chi co 48 kich ban (seed 1).
%   Chi so moi chuyen bay:
%     n_ok       so nhiem vu dat (dung han, m <= 1) truoc khi het H2 / SoC < 30 % (chinh)
%     H2_per_ok  H2 tuong duong (hieu chinh SoC) tren moi nhiem vu dat (g)
%     n_viol, viol_time, crash, t_flight, soc_end, bat_dmg (%), fc_uV (uV/cell)
%   Bao cao (indir/report): policy_summary.csv, paired_vs_ours.csv, by_wind.csv,
%   by_config.csv, task_ok_by_type.csv, onpolicy_afp.csv, summary.md, policies.png.
root = fileparts(fileparts(mfilename('fullpath')));
if nargin < 1, indir = fullfile(root, 'results', 'E3'); end
out = fullfile(indir, 'report'); if ~exist(out, 'dir'), mkdir(out); end
S = load(fullfile(indir, 'all_E3.mat'));
T = S.T; tr = S.traces;
T.crash = T.stop_reason == "crash";
T.ok = T.ok == 1;
mets = {'n_ok', 'H2_per_ok', 'n_viol', 'viol_time', 't_flight', 'soc_end', 'bat_dmg', 'fc_uV', 'n_done'};
better_low = {'H2_per_ok', 'n_viol', 'viol_time', 'bat_dmg', 'fc_uV'};
pols = unique(T.in_policy, 'stable');

% ---- Tong hop theo chinh sach
rows = {};
for i = 1:numel(pols)
    t = T(T.in_policy == pols(i), :);
    r.policy = pols(i); r.n = height(t); r.fail = mean(~t.ok); r.crash = mean(t.crash);
    r.stop_h2 = mean(t.stop_reason == "h2"); r.stop_soc = mean(t.stop_reason == "soc");
    for m = mets
        x = t.(m{1}); r.([m{1} '_mean']) = mean(x, 'omitnan'); r.([m{1} '_med']) = median(x, 'omitnan');
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
        if ismember(m{1}, better_low), r.ours_better = mean(d > 0); else, r.ours_better = mean(d < 0); end
        rows{end+1} = r; %#ok<AGROW>
    end
end
R.paired = struct2table([rows{:}]');
writetable(R.paired, fullfile(out, 'paired_vs_ours.csv'));

% ---- Theo muc gio va cau hinh
R.by_wind = groupsummary(T, {'in_policy', 'in_W_lvl'}, {'mean'}, {'n_ok', 'n_viol', 'H2_per_ok'});
writetable(R.by_wind, fullfile(out, 'by_wind.csv'));
R.by_config = groupsummary(T, {'in_policy', 'in_fc_wkg', 'in_dx_cg'}, {'mean'}, {'n_ok', 'n_viol', 'H2_per_ok'});
writetable(R.by_config, fullfile(out, 'by_config.csv'));

% ---- Ti le nhiem vu dat theo loai
names = {'transit', 'turn', 'climb', 'loiter', 'survey', 'gust_hold'};
rows = {};
for i = 1:numel(pols)
    sel = find(T.in_policy == pols(i));
    TK = [];
    for j = sel.'
        if isstruct(tr{j}) && isfield(tr{j}, 'tasks'), TK = [TK; double(tr{j}.tasks)]; end %#ok<AGROW>
    end
    if isempty(TK), continue; end
    TK = TK(TK(:, 17) == 0, :);                       % bo nhiem vu bi cat do dung chuyen bay
    r = struct('policy', pols(i));
    for k = 1:6
        q = TK(:, 2) == k;
        r.([names{k} '_ok']) = mean(TK(q, 9)); r.([names{k} '_viol']) = mean(TK(q, 10) > 1);
    end
    rows{end+1} = r; %#ok<AGROW>
end
R.task = struct2table([rows{:}]');
writetable(R.task, fullfile(out, 'task_ok_by_type.csv'));

% ---- AFP tren quy dao cua chinh sach (cua so T_plan, khong phai chan troi 10 s)
rows = {};
for pname = ["ours", "A2_no_conf", "A10_global_conf", "A3_no_config", "A4_no_wind", "A11_no_elec"]
    sel = find(T.in_policy == pname);
    DL = [];
    for j = sel.'
        if isstruct(tr{j}) && isfield(tr{j}, 'decisions'), DL = [DL; double(tr{j}.decisions)]; end %#ok<AGROW>
    end
    if isempty(DL), continue; end
    ub = DL(:, 6); m = DL(:, 8);
    rows{end+1} = struct('policy', pname, 'n_dec', numel(m), 'frac_ub_le1', mean(ub <= 1), ...
        'viol_rate', mean(m > 1), 'viol_given_ub_le1', mean(m(ub <= 1) > 1), ...
        'nfeas_med', median(DL(:, 10))); %#ok<AGROW>
end
if ~isempty(rows)
    R.onpolicy = struct2table([rows{:}]');
    writetable(R.onpolicy, fullfile(out, 'onpolicy_afp.csv'));
end

% ---- Hinh
fig = figure('Visible', 'off', 'Position', [100 100 1100 420]);
s = R.summary; [~, o] = sort(s.n_ok_mean, 'descend'); s = s(o, :);
subplot(1, 2, 1); barh(categorical(s.policy, s.policy), s.n_ok_mean); xlabel('so nhiem vu dat (trung binh)'); grid on
subplot(1, 2, 2); barh(categorical(s.policy, s.policy), s.n_viol_mean); xlabel('so nhiem vu vi pham (trung binh)'); grid on
exportgraphics(fig, fullfile(out, 'policies.png'), 'Resolution', 150);
close(fig);

% ---- Tom tat
fid = fopen(fullfile(out, 'summary.md'), 'w');
fprintf(fid, '# E3: so sanh chinh sach (bay den khi het H2 hoac SoC < 30 %%)\n\n');
fprintf(fid, '%d chuyen bay; %d chinh sach/bien the.\n\n', height(T), numel(pols));
fprintf(fid, '| chinh sach | n | loi | roi | n_ok TB | n_ok TV | H2/nv dat (g) | n_viol TB | t bay TB (s) | SoC cuoi TB |\n|---|---|---|---|---|---|---|---|---|---|\n');
for i = 1:height(s)
    fprintf(fid, '| %s | %d | %.3f | %.3f | %.2f | %.1f | %.3f | %.2f | %.0f | %.3f |\n', s.policy(i), s.n(i), ...
        s.fail(i), s.crash(i), s.n_ok_mean(i), s.n_ok_med(i), s.H2_per_ok_mean(i), s.n_viol_mean(i), ...
        s.t_flight_mean(i), s.soc_end_mean(i));
end
fprintf(fid, '\n## So sanh cap voi ours (n_ok, n_viol, H2_per_ok)\n\n| chinh sach | chi so | cap | TB(chinh sach - ours) | p Wilcoxon | ti le ours tot hon |\n|---|---|---|---|---|---|\n');
P = R.paired(ismember(R.paired.metric, ["n_ok", "n_viol", "H2_per_ok"]), :);
for i = 1:height(P)
    fprintf(fid, '| %s | %s | %d | %.3f | %.2g | %.2f |\n', P.policy(i), P.metric(i), P.n_pairs(i), ...
        P.mean_diff(i), P.p_signrank(i), P.ours_better(i));
end
fclose(fid);
fprintf('E3: phan tich xong, bao cao trong %s\n', out);
end
