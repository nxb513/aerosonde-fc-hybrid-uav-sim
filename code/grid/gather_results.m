function [T, traces, ckpt] = gather_results(exp, indir)
%GATHER_RESULTS Gop ket qua cac job thanh mot bang (sap theo in_id).
%   [T, traces, ckpt] = GATHER_RESULTS(exp, indir); luu all_<exp>.mat vao indir.
%   Dung chunk_XX.mat; job nao khong co chunk (loi/het gio) thi lay phan da xong tu
%   ckpt_XX.mat. ckpt: bang nhat ky checkpoint [job, so kich ban, thoi gian, id cuoi].
if nargin < 2
    root = fileparts(fileparts(mfilename('fullpath')));
    indir = fullfile(root, 'results', exp);
end
fc = dir(fullfile(indir, '**', 'chunk_*.mat'));
fk = dir(fullfile(indir, '**', 'ckpt_*.mat'));
jobs_c = cellfun(@(s) sscanf(s, 'chunk_%d.mat'), {fc.name});
Ts = {}; Tr = {}; ck = zeros(0, 4);
for k = 1:numel(fc)
    D = load(fullfile(fc(k).folder, fc(k).name));
    if isempty(D.T), continue; end
    Ts{end+1} = D.T; Tr{end+1} = D.traces(:); %#ok<AGROW>
    if isfield(D, 'ck_log'), ck = [ck; [repmat(jobs_c(k), size(D.ck_log, 1), 1), D.ck_log]]; end %#ok<AGROW>
end
for k = 1:numel(fk)
    j = sscanf(fk(k).name, 'ckpt_%d.mat');
    if any(jobs_c == j), continue; end
    D = load(fullfile(fk(k).folder, fk(k).name));
    Ts{end+1} = rows_to_table(D.rows(1:D.n_done)); Tr{end+1} = D.traces(1:D.n_done); %#ok<AGROW>
    ck = [ck; [repmat(j, size(D.ck_log, 1), 1), D.ck_log]]; %#ok<AGROW>
    fprintf('Job %d: dung checkpoint (%d kich ban).\n', j, D.n_done);
end
T = vertcat(Ts{:});
traces = vertcat(Tr{:});
[~, o] = sort(T.in_id);
T = T(o, :);
if ~isempty(traces), traces = traces(o); end
ckpt = array2table(ck, 'VariableNames', {'job', 'n_done', 't_s', 'id_last'});
save(fullfile(indir, sprintf('all_%s.mat', exp)), 'T', 'traces', 'ckpt', '-v7');
writetable(T, fullfile(indir, sprintf('all_%s.csv', exp)));
fprintf('%s: %d kich ban; ok = %d; %d checkpoint\n', exp, height(T), nnz(T.ok), height(ckpt));
end
