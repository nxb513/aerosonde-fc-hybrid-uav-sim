%DEMO_TRIM_MARGIN Bien elevon con lai khi trim theo khoi luong them va dich CG.
%   Tai them dat tai x_cg (so voi CG danh dinh), bay ngang Va = 18 m/s.
%   Ket qua: bang trong cua so lenh + hinh results/trim_margin.png.
%   Day la vi du minh hoa; dai khoi luong/CG chua phai cau hinh FC da chot.

setup_paths();
p0 = x8_params();
d2r = pi/180;
Va = 18;
dm   = [0 0.5 1.0 1.5];          % kg them
xpos = (-0.10:0.05:0.10);        % m, vi tri dat khoi luong them (duong = phia mui)

margin = nan(numel(dm), numel(xpos));
alpha  = margin; Pel = margin; feas = false(size(margin));
for i = 1:numel(dm)
    for j = 1:numel(xpos)
        p = x8_apply_config(p0, struct('name', 'tai', 'm', dm(i), 'r', [xpos(j); 0; 0], 'J', []));
        tr = x8_trim(p, Va, 0);
        margin(i, j) = tr.elevon_margin/d2r;
        alpha(i, j)  = tr.alpha/d2r;
        Pel(i, j)    = tr.Pel;
        feas(i, j)   = tr.feasible;
    end
end

fprintf('\nBien elevon con lai (deg) tai Va = %.0f m/s. Hang: khoi luong them (kg); cot: vi tri x (m)\n', Va);
disp(array2table(margin, 'RowNames', compose('+%.1f kg', dm), ...
    'VariableNames', compose('x=%+.2f', xpos)));
fprintf('Alpha trim (deg):\n');
disp(array2table(alpha, 'RowNames', compose('+%.1f kg', dm), ...
    'VariableNames', compose('x=%+.2f', xpos)));
fprintf('Cong suat dien khi trim (W):\n');
disp(array2table(Pel, 'RowNames', compose('+%.1f kg', dm), ...
    'VariableNames', compose('x=%+.2f', xpos)));
fprintf('Kha thi (elevon, ga, alpha trong vung hop le): %d / %d diem\n', nnz(feas), numel(feas));

root = fileparts(fileparts(mfilename('fullpath')));
outdir = fullfile(root, 'results');
if ~exist(outdir, 'dir'), mkdir(outdir); end
f = figure('Visible', 'off', 'Position', [100 100 900 360]);
subplot(1, 2, 1);
plot(xpos*100, margin, '-o', 'LineWidth', 1.2); grid on;
xlabel('Vi tri tai them x (cm, duong = phia mui)'); ylabel('Bien elevon con lai (deg)');
legend(compose('+%.1f kg', dm), 'Location', 'best'); title(sprintf('Trim ngang, V_a = %d m/s', Va));
subplot(1, 2, 2);
plot(xpos*100, alpha, '-o', 'LineWidth', 1.2); grid on; hold on;
yline(p0.aero.alpha_valid(2)/d2r, '--k', 'alpha hop le toi da [R1]');
xlabel('Vi tri tai them x (cm)'); ylabel('\alpha trim (deg)');
exportgraphics(f, fullfile(outdir, 'trim_margin.png'), 'Resolution', 150);
close(f);
fprintf('Da luu %s\n', fullfile(outdir, 'trim_margin.png'));
