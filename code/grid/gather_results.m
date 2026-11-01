function [T, traces] = gather_results(exp, indir)
%GATHER_RESULTS Gop cac chunk_*.mat cua thi nghiem thanh mot bang (sap theo in_id).
%   [T, traces] = GATHER_RESULTS(exp, indir); luu all_<exp>.mat vao indir.
if nargin < 2
    root = fileparts(fileparts(mfilename('fullpath')));
    indir = fullfile(root, 'results', exp);
end
f = dir(fullfile(indir, '**', 'chunk_*.mat'));
Ts = cell(numel(f), 1); Tr = cell(numel(f), 1);
for k = 1:numel(f)
    D = load(fullfile(f(k).folder, f(k).name));
    Ts{k} = D.T; Tr{k} = D.traces(:);
end
T = vertcat(Ts{:});
traces = vertcat(Tr{:});
[~, o] = sort(T.in_id);
T = T(o, :);
if ~isempty(traces), traces = traces(o); end
save(fullfile(indir, sprintf('all_%s.mat', exp)), 'T', 'traces', '-v7');
writetable(T, fullfile(indir, sprintf('all_%s.csv', exp)));
fprintf('%s: %d kich ban tu %d chunk; ok = %d\n', exp, height(T), numel(f), nnz(T.ok));
end
