function D = afp_dataset(T)
%AFP_DATASET Dac trung va nhan cho AFP tu bang ket qua (E1/E2).
%   D = AFP_DATASET(T)
%   Dac trung chia nhom (de lam ablation):
%     traj    phi_max, gamma_max, Va, dchi, dh            (quy dao ung vien)
%     config  mass, x_cg, Jy, P_fc                         (khoi luong/CG, A3)
%     wind    W_h, turb, cos/sin huong gio tuong doi      (gio, A4)
%     phys    alpha, de, bien elevon, Pel tai trim        (tinh truoc chuyen bay)
%   Nhan: m = max(m_cmd, alpha_p99/12 deg). m > 1 la vi pham (lenh elevon vuot
%   +-30 deg [R7] hoac alpha vuot vung tuyen tinh 12 deg [R1]). Kich ban khong
%   trim duoc hoac phan ky gan m = 2 (vi pham chac chan).
ok = T.ok & isfinite(T.m_cmd);
m = max(T.m_cmd, T.alpha_p99/12);
m(~ok) = 2;
rel = T.in_wind_dir - T.in_dchi;          % huong gio so voi huong ket thuc
X = [T.in_phi_max, T.in_gamma_max, T.in_Va, T.in_dchi, T.in_dh, ...
     T.mass, T.x_cg, T.Jy, T.in_P_fc, ...
     T.in_W_h, T.in_turb, cosd(rel), sind(rel), ...
     T.trim_alpha, T.trim_de, T.trim_margin, T.trim_Pel];
names = {'phi_max','gamma_max','Va','dchi','dh', 'mass','x_cg','Jy','P_fc', ...
         'W_h','turb','cos_rel','sin_rel', 'trim_alpha','trim_de','trim_margin','trim_Pel'};
D.X = X; D.names = names; D.m = m; D.viol = m > 1;
D.groups = struct('traj', 1:5, 'config', 6:9, 'wind', 10:13, 'phys', 14:17);
D.wind_bin = discretize(T.in_W_h, [-inf 3 7 10 inf]);   % nhom cho conformal theo nhom
D.ok = ok;
end
