function D = afp_dataset(T)
%AFP_DATASET Dac trung va nhan cho AFP tu bang ket qua (E1/E2).
%   Nhom dac trung (de lam ablation):
%     traj    phi_max, gamma_max, Va, dchi, dh               (quy dao ung vien)
%     config  mass, x_cg, SM, Jy, P_fc                         (khoi luong/CG, A3)
%     wind    W_h, turb, cos/sin huong gio tuong doi         (gio, A4)
%     actu    dmax, rate, delay, alpha0                       (gioi han co cau/stall)
%     phys    alpha, de, ga, bien lai, Pel tai trim          (tinh truoc chuyen bay)
%   Nhan: m = max(m_cmd, alpha_p99/alpha_lim); m > 1 la vi pham (lenh lai vuot gioi han
%   hoac alpha vuot bien 2 deg truoc stall). Kich ban khong trim duoc/phan ky: m = 2.
ok = T.ok & isfinite(T.m_cmd);
m = max(T.m_cmd, T.alpha_p99./T.alpha_lim);
m(~ok) = 2;
rel = T.in_wind_dir - T.in_dchi;
X = [T.in_phi_max, T.in_gamma_max, T.in_Va, T.in_dchi, T.in_dh, ...
     T.mass, T.x_cg, T.SM, T.Jy, T.in_P_fc, ...
     T.in_W_h, T.in_turb, cosd(rel), sind(rel), ...
     T.in_dmax_deg, T.in_rate_deg, T.in_delay, T.in_alpha0_deg, ...
     T.trim_alpha, T.trim_de, T.trim_dt, T.trim_margin, T.trim_Pel];
names = {'phi_max','gamma_max','Va','dchi','dh', 'mass','x_cg','SM','Jy','P_fc', ...
         'W_h','turb','cos_rel','sin_rel', 'dmax','rate','delay','alpha0', ...
         'trim_alpha','trim_de','trim_dt','trim_margin','trim_Pel'};
X(~isfinite(X)) = 0;
D.X = X; D.names = names; D.m = m; D.viol = m > 1;
D.groups = struct('traj', 1:5, 'config', 6:10, 'wind', 11:14, 'actu', 15:18, 'phys', 19:23);
D.wind_bin = discretize(T.in_W_h, [-inf 3 7 10 inf]);
D.ok = ok;
end
