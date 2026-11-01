function e3_train_all(indir, outdir)
%E3_TRAIN_ALL Tu du lieu tham do E3X: huan luyen bo mo hinh cho planner + learning curve.
%   E3_TRAIN_ALL(indir, outdir); mac dinh indir = results/E3X (chua all_E3X.mat),
%   outdir = results/E3 (ghi models.mat). Bao cao: indir/report/
%     afp_test.csv        chi so AFP va R^2 mo hinh thay the tren tap kiem tra, moi bien the
%     learning_curve.*    duong hoc AFP + checkpoint mo hinh (e3_learning_curve)
%     summary.md          tom tat
root = fileparts(fileparts(mfilename('fullpath')));
if nargin < 1, indir = fullfile(root, 'results', 'E3X'); end
if nargin < 2, outdir = fullfile(root, 'results', 'E3'); end
rep_dir = fullfile(indir, 'report');
if ~exist(rep_dir, 'dir'), mkdir(rep_dir); end
if ~exist(outdir, 'dir'), mkdir(outdir); end
S = load(fullfile(indir, 'all_E3X.mat'));
D = e3_dataset(S.traces);
fprintf('E3X: %d chuyen bay, %d cua so, ti le vi pham %.3f, roi %d\n', ...
    numel(unique(D.flight)), numel(D.m), mean(D.viol), nnz(D.crash));
[M, rep] = e3_train(D);
save(fullfile(outdir, 'models.mat'), '-struct', 'M', '-v7');
writetable(rep.test, fullfile(rep_dir, 'afp_test.csv'));
LC = e3_learning_curve(D, M.split, rep_dir);

fid = fopen(fullfile(rep_dir, 'summary.md'), 'w');
fprintf(fid, '# E3X: du lieu tham do va mo hinh\n\n');
fprintf(fid, '- %d chuyen bay, %d cua so (>= 2 s); ti le vi pham %.3f; cua so co roi %d\n', ...
    numel(unique(D.flight)), numel(D.m), mean(D.viol), nnz(D.crash));
fprintf(fid, '- Chia theo chuyen bay: %d / %d / %d (huan luyen / hieu chuan / kiem tra)\n\n', ...
    numel(M.split.f_tr), numel(M.split.f_ca), numel(M.split.f_te));
fprintf(fid, '## AFP va mo hinh thay the tren tap kiem tra\n\n');
write_md(fid, rep.test);
fprintf(fid, '\n## Learning curve (theo thu tu thu du lieu)\n\n');
write_md(fid, LC(LC.rep == 0, :));
fclose(fid);
end

function write_md(fid, T)
v = T.Properties.VariableNames;
fprintf(fid, '| %s |\n', strjoin(v, ' | '));
fprintf(fid, '|%s\n', repmat('---|', 1, numel(v)));
for i = 1:height(T)
    c = cell(1, numel(v));
    for j = 1:numel(v)
        x = T{i, j};
        if isstring(x) || ischar(x), c{j} = char(x); else, c{j} = sprintf('%.4g', x); end
    end
    fprintf(fid, '| %s |\n', strjoin(c, ' | '));
end
end
