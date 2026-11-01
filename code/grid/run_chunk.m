function run_chunk(exp, job, njobs, outdir, models_file)
%RUN_CHUNK Chay phan job/njobs cua thi nghiem exp, luu results/<exp>/chunk_<job>.mat.
%   RUN_CHUNK(exp, job, njobs, outdir, models_file)
%   Chia vong tron (i = job, job+njobs, ...) de can bang tai giua cac job.
%   Moi chuyen bay boc try/catch; loi ghi vao cot msg.
%   Checkpoint (de khong mat du lieu khi job bi dung, va de ve learning curve theo luong
%   du lieu tich luy):
%     ckpt_<job>.mat   cac chuyen bay da xong + nhat ky [so chuyen, thoi gian (s), id cuoi],
%                      ghi sau moi chuyen bay; neu ton tai thi chay tiep tu do
%     fckpt_<job>.mat  trang thai chuyen bay dang bay, ghi moi 5 phut tinh (run_flight)
%   models_file: bo mo hinh (e3_train) cho thi nghiem 'E3' (mac dinh results/E3/models.mat).
root = fileparts(fileparts(mfilename('fullpath')));
if nargin < 4 || isempty(outdir), outdir = fullfile(root, 'results', exp); end
if nargin < 5 || isempty(models_file), models_file = fullfile(root, 'results', 'E3', 'models.mat'); end
if ~exist(outdir, 'dir'), mkdir(outdir); end
S = grid_def(exp);
idx = job:njobs:numel(S);
if isempty(idx)
    T = table(); traces = {}; ck_log = zeros(0, 3); %#ok<NASGU>
    save(fullfile(outdir, sprintf('chunk_%02d.mat', job)), 'T', 'traces', 'idx', 'ck_log', '-v7');
    fprintf('Job %d khong co chuyen bay.\n', job);
    return
end
M = [];
if any(arrayfun(@(s) ~any(strcmp(s.policy, {'explore','fixed_gentle','fixed_nominal', ...
        'fixed_aggr','nominal_rule','nominal_ecms'})), S(idx)))
    M = load(models_file);
end
rows = cell(numel(idx), 1); traces = cell(numel(idx), 1);
ckfile = fullfile(outdir, sprintf('ckpt_%02d.mat', job));
fkfile = fullfile(outdir, sprintf('fckpt_%02d.mat', job));
n0 = 0; ck_log = zeros(0, 3);
if exist(ckfile, 'file')
    C = load(ckfile);
    n0 = C.n_done; rows(1:n0) = C.rows(1:n0); traces(1:n0) = C.traces(1:n0); ck_log = C.ck_log;
    fprintf('Chay tiep tu checkpoint: %d/%d chuyen bay.\n', n0, numel(idx));
end
t0 = tic;
for n = n0 + 1:numel(idx)
    fl = S(idx(n));
    try
        r = run_flight(fl, M, struct('ckpt_file', fkfile, 'ckpt_every', 300));
    catch err
        r = struct('ok', false, 'msg', ['error: ' err.message]);
    end
    if isfield(r, 'trace'), traces{n} = r.trace; r = rmfield(r, 'trace'); end
    rows{n} = flatten(fl, r);
    ck_log(end+1, :) = [n, toc(t0), idx(n)]; %#ok<AGROW>
    n_done = n; %#ok<NASGU>
    save(ckfile, 'rows', 'traces', 'idx', 'n_done', 'ck_log', '-v7');
    fprintf('[%s job %d] %d/%d xong (id %d, %s), %.0f s\n', exp, job, n, numel(idx), ...
        fl.id, fl.policy, toc(t0));
end
T = rows_to_table(rows(1:numel(idx)));
file = fullfile(outdir, sprintf('chunk_%02d.mat', job));
save(file, 'T', 'traces', 'idx', 'ck_log', '-v7');
fprintf('Da luu %s (%d dong, %.1f phut)\n', file, height(T), toc(t0)/60);
end

function row = flatten(sc, r)
row = struct();
f = fieldnames(sc);
for k = 1:numel(f)
    v = sc.(f{k});
    if ischar(v), v = string(v); end
    row.(['in_' f{k}]) = v;
end
f = fieldnames(r);
for k = 1:numel(f)
    v = r.(f{k});
    if ischar(v), v = string(v); end
    if isscalar(v) || isstring(v), row.(f{k}) = v; end
end
end
