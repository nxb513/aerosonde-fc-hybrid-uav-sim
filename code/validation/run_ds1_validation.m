function T = run_ds1_validation(rho)
%RUN_DS1_VALIDATION Kiem chung mo hinh X8 voi 17 maneuver cua [DS1].
%   T = RUN_DS1_VALIDATION(rho) chay mo phong ho vong cho tung maneuver,
%   in bang sai so va luu results/ds1_validation_rho*.csv cung hinh cho 2
%   maneuver kiem chung. rho mac dinh = p.env.rho.
%   Chi so: RMSE va he so tuong quan (corr) giua mo phong va do.

setup_paths();
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'validation'));
p = x8_params();
if nargin >= 1 && ~isempty(rho), p.env.rho = rho; end
dt = 0.0025;

files = [dir(fullfile(root, '..', 'data', 'DS1', 'training', '*.mat'));
         dir(fullfile(root, '..', 'data', 'DS1', 'validation', '*.mat'))];
r2d = 180/pi;
vars  = {'alpha', 'theta', 'q', 'Va', 'beta', 'phi', 'p', 'r'};
scale = [r2d r2d r2d 1 r2d r2d r2d r2d];
rows = cell(numel(files), 1);
M = nan(numel(files), 2*numel(vars) + 5);
res = cell(numel(files), 1);
for k = 1:numel(files)
    d = ds1_load(fullfile(files(k).folder, files(k).name));
    r = ds1_simulate_maneuver(p, d, dt);
    res{k} = r;
    rows{k} = sprintf('%s/%s', regexprep(files(k).folder, '.*[\\/]', ''), d.name);
    for j = 1:numel(vars)
        a = r.sim.(vars{j})*scale(j); b = r.meas.(vars{j})*scale(j);
        M(k, 2*j-1) = sqrt(mean((a - b).^2));
        c = corrcoef(a, b); M(k, 2*j) = c(1, 2);
    end
    Im = r.meas.Ibus; Iq = r.Ibus_quasi; Is = r.sim.Ibus;
    M(k, end-4:end) = [mean(Im), mean(Iq), mean(Is), ...
        sqrt(mean((Iq - Im).^2)), (mean(Iq) - mean(Im))/mean(Im)*100];
end
cols = [reshape([strcat(vars, '_rmse'); strcat(vars, '_corr')], 1, []), ...
        {'I_meas_mean', 'I_quasi_mean', 'I_sim_mean', 'I_quasi_rmse', 'I_quasi_bias_pct'}];
T = array2table(M, 'RowNames', rows, 'VariableNames', cols);

fprintf('\nrho = %.3f kg/m^3. Don vi RMSE: deg, deg/s, m/s, A.\n', p.env.rho);
disp(T(:, {'alpha_rmse','alpha_corr','theta_rmse','q_rmse','q_corr','Va_rmse', ...
           'beta_rmse','phi_rmse','p_rmse','p_corr','r_rmse','r_corr'}));
disp(T(:, {'I_meas_mean','I_quasi_mean','I_sim_mean','I_quasi_rmse','I_quasi_bias_pct'}));

outdir = fullfile(root, 'results');
if ~exist(outdir, 'dir'), mkdir(outdir); end
writetable(T, fullfile(outdir, sprintf('ds1_validation_rho%.3f.csv', p.env.rho)), 'WriteRowNames', true);

for name = {'longitudinal_3211_5', 'lateral_121_4'}
    k = find(contains(rows, name{1}), 1);
    plot_case(res{k}, fullfile(outdir, sprintf('ds1_%s_rho%.3f.png', name{1}, p.env.rho)));
end
end

function plot_case(r, file)
r2d = 180/pi;
f = figure('Visible', 'off', 'Position', [100 100 1000 700]);
v = {'alpha', 'theta', 'q', 'Va', 'phi', 'p', 'r', 'Ibus'};
lab = {'\alpha (deg)', '\theta (deg)', 'q (deg/s)', 'V_a (m/s)', '\phi (deg)', 'p (deg/s)', 'r (deg/s)', 'I_{pin} (A)'};
sc = [r2d r2d r2d 1 r2d r2d r2d 1];
for j = 1:numel(v)
    subplot(4, 2, j);
    plot(r.t, r.meas.(v{j})*sc(j), 'k', r.t, r.sim.(v{j})*sc(j), 'b', 'LineWidth', 1); hold on;
    if strcmp(v{j}, 'Ibus'), plot(r.t, r.Ibus_quasi, 'r--', 'LineWidth', 1); end
    grid on; ylabel(lab{j});
    if j == 1, legend('do [DS1]', 'mo phong', 'Location', 'best'); title(strrep(r.name, '_', '\_')); end
    if j == 8, legend('do', 'mo phong', 'gan tinh', 'Location', 'best'); end
    if j >= 7, xlabel('t (s)'); end
end
exportgraphics(f, file, 'Resolution', 130);
close(f);
end
