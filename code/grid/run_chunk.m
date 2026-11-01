function run_chunk(exp, job, njobs, outdir, ckpt_every)
%RUN_CHUNK Chay phan job/njobs cua thi nghiem exp, luu results/<exp>/chunk_<job>.mat.
%   Chia vong tron (i = job, job+njobs, ...) de can bang tai giua cac job.
%   Moi kich ban duoc boc try/catch; loi ghi vao cot msg.
%   E2 luu them chuoi P_load (single, 20 Hz) de danh gia lai EMS ngoai vong.
%   Checkpoint: cu ckpt_every kich ban (mac dinh 50) ghi ckpt_<job>.mat gom cac dong
%   da xong, so kich ban va thoi gian; neu file da ton tai thi chay tiep tu do.
%   Dung de (i) khong mat du lieu khi job bi dung, (ii) ve learning curve theo
%   luong du lieu tich luy (ai/learning_curve).
if nargin < 4 || isempty(outdir)
    root = fileparts(fileparts(mfilename('fullpath')));
    outdir = fullfile(root, 'results', exp);
end
if nargin < 5, ckpt_every = 50; end
if ~exist(outdir, 'dir'), mkdir(outdir); end
S = grid_def(exp);
idx = job:njobs:numel(S);
if isempty(idx)
    T = table(); traces = {}; %#ok<NASGU>
    save(fullfile(outdir, sprintf('chunk_%02d.mat', job)), 'T', 'traces', 'idx', '-v7');
    fprintf('Job %d khong co kich ban.\n', job);
    return
end
keep_trace = ~strcmp(exp, 'E1');
rows = cell(numel(idx), 1); traces = cell(numel(idx), 1);
ckfile = fullfile(outdir, sprintf('ckpt_%02d.mat', job));
n0 = 0; ck_log = zeros(0, 3);            % [so kich ban da xong, thoi gian (s), id cuoi]
if exist(ckfile, 'file')
    C = load(ckfile);
    n0 = C.n_done; rows(1:n0) = C.rows(1:n0); traces(1:n0) = C.traces(1:n0); ck_log = C.ck_log;
    fprintf('Chay tiep tu checkpoint: %d/%d kich ban.\n', n0, numel(idx));
end
t0 = tic;
for n = n0 + 1:numel(idx)
    sc = S(idx(n));
    try
        r = run_scenario(sc);
    catch err
        r = struct('ok', false, 'msg', ['error: ' err.message]);
    end
    if isfield(r, 'trace')
        if keep_trace
            traces{n} = struct('P', single(r.trace.P), 'V', single(r.trace.V), ...
                               'Pfc', single(r.trace.Pfc), 'Psc', single(r.trace.Psc));
        end
        r = rmfield(r, 'trace');
    end
    rows{n} = flatten(sc, r);
    if mod(n, ckpt_every) == 0 || n == numel(idx)
        ck_log(end+1, :) = [n, toc(t0), idx(n)]; %#ok<AGROW>
        n_done = n; %#ok<NASGU>
        save(ckfile, 'rows', 'traces', 'idx', 'n_done', 'ck_log', '-v7');
        fprintf('[%s job %d] checkpoint %d/%d, %.1f s/kich ban\n', exp, job, n, numel(idx), ...
            toc(t0)/max(n - n0, 1));
    end
end
T = rows_to_table(rows);
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
