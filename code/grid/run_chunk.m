function run_chunk(exp, job, njobs, outdir)
%RUN_CHUNK Chay phan job/njobs cua thi nghiem exp, luu results/<exp>/chunk_<job>.mat.
%   Chia vong tron (i = job, job+njobs, ...) de can bang tai giua cac job.
%   Moi kich ban duoc boc try/catch; loi ghi vao cot msg.
%   E2 luu them chuoi P_load (single, 20 Hz) de danh gia lai EMS ngoai vong.
if nargin < 4
    root = fileparts(fileparts(mfilename('fullpath')));
    outdir = fullfile(root, 'results', exp);
end
if ~exist(outdir, 'dir'), mkdir(outdir); end
S = grid_def(exp);
idx = job:njobs:numel(S);
keep_trace = ~strcmp(exp, 'E1');
rows = cell(numel(idx), 1); traces = cell(numel(idx), 1);
t0 = tic;
for n = 1:numel(idx)
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
    if mod(n, 25) == 0
        fprintf('[%s job %d] %d/%d, %.1f s/kich ban\n', exp, job, n, numel(idx), toc(t0)/n);
    end
end
T = struct2table(fill_fields(rows));
file = fullfile(outdir, sprintf('chunk_%02d.mat', job));
save(file, 'T', 'traces', 'idx', '-v7');
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

function rows = fill_fields(rows)
% Dong nhat ten truong giua cac dong (kich ban loi co it truong hon)
allf = {};
for k = 1:numel(rows), allf = union(allf, fieldnames(rows{k}), 'stable'); end
for k = 1:numel(rows)
    for j = 1:numel(allf)
        if ~isfield(rows{k}, allf{j})
            if strcmp(allf{j}, 'msg'), rows{k}.(allf{j}) = ""; else, rows{k}.(allf{j}) = NaN; end
        end
    end
    rows{k} = orderfields(rows{k}, allf);
end
rows = [rows{:}]';
end
