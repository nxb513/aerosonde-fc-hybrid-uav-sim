function make_mini(outdir)
%MAKE_MINI Tao bo du lieu nho (E1: 160 mau, E2: 3 tinh huong x 36 ung vien) de chay
%   thu chuoi phan tich tai may truoc khi co ket qua luoi day du.
if nargin < 1
    root = fileparts(fileparts(mfilename('fullpath')));
    outdir = fullfile(root, 'results', 'mini');
end
S1 = grid_def('E1'); S1 = S1(1:160);
S2 = grid_def('E2');
key = [[S2.P_fc]', [S2.dx_cg]', [S2.W_h]', [S2.turb]', [S2.dchi]', [S2.dh]', [S2.aero_seed]'];
[~, ~, sit] = unique(key, 'rows');
pick = ismember(sit, [1 400 800]);
S2 = S2(pick);
for e = {'E1', 'E2'}
    if strcmp(e{1}, 'E1'), S = S1; else, S = S2; end
    rows = cell(numel(S), 1); traces = cell(numel(S), 1);
    for i = 1:numel(S)
        r = run_scenario(S(i));
        if isfield(r, 'trace')
            traces{i} = struct('P', single(r.trace.P)); r = rmfield(r, 'trace');
        end
        rows{i} = [fieldnames_prefixed(S(i)); struct2cell_pairs(r)];
    end
    T = to_table(rows); %#ok<NASGU>
    if ~exist(fullfile(outdir, e{1}), 'dir'), mkdir(fullfile(outdir, e{1})); end
    idx = 1:numel(S); %#ok<NASGU>
    save(fullfile(outdir, e{1}, 'chunk_01.mat'), 'T', 'traces', 'idx');
end
end

function c = fieldnames_prefixed(s)
f = fieldnames(s); c = cell(numel(f), 2);
for k = 1:numel(f)
    v = s.(f{k}); if ischar(v), v = string(v); end
    c(k, :) = {['in_' f{k}], v};
end
end

function c = struct2cell_pairs(r)
f = fieldnames(r); c = cell(0, 2);
for k = 1:numel(f)
    v = r.(f{k}); if ischar(v), v = string(v); end
    if isscalar(v) || isstring(v), c(end+1, :) = {f{k}, v}; end %#ok<AGROW>
end
end

function T = to_table(rows)
allf = {};
for k = 1:numel(rows), allf = union(allf, rows{k}(:, 1), 'stable'); end
S = struct();
for k = 1:numel(rows)
    for j = 1:numel(allf)
        i = find(strcmp(rows{k}(:, 1), allf{j}), 1);
        if isempty(i), v = NaN; if strcmp(allf{j}, 'msg'), v = ""; end, else, v = rows{k}{i, 2}; end
        S(k).(allf{j}) = v;
    end
end
T = struct2table(S(:));
end
