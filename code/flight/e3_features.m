function [X, names, groups] = e3_features(ctx, traj)
%E3_FEATURES Ma tran dac trung cho cac kieu bay tai mot thoi diem quyet dinh (32 cot).
%   [X, names, groups] = E3_FEATURES(ctx, traj)
%     ctx   boi canh luc quyet dinh (run_flight); traj (n x 3) [phi_max gamma_max Va]
%   Nhom dac trung (de lam ablation):
%     cand    goc nghieng cho phep, goc leo cho phep, van toc               (3)
%     task    loai nhiem vu, sai lech huong, do cao con lai, sai lech ngang, do cong,
%             khoang cach toi diem chuyen chang, goc doi huong ke tiep       (7)
%     config  m, x_cg, SM, Jy, P_fc, co sieu tu, Td, f_c                     (8, A4)
%     wind    gio trung binh, he so nhieu, cos/sin huong gio so voi huong bay (4, A5)
%     state   Va, phi, gamma, alpha, muc dung lai 6 s gan nhat               (5)
%     elec    cong suat dien dong co, SoC, nang luong sieu tu, P_FC, U_bus   (5, A6)
%   Td va f_c co dinh trong mot chuyen bay; co trong dac trung de mo hinh dung duoc cho
%   cac ban do nhay S1, S6. Goi khong doi so: chi tra ve names, groups.
names = {'phi_max','gamma_max','Va_c', ...
         'task','e_chi','dh_rem','e_py','kappa','d_wp','turn_next', ...
         'mass','x_cg','SM','Jy','P_fc','sc_use','Td','f_c', ...
         'W','turb','cos_rel','sin_rel', ...
         'Va','phi','gamma','alpha','u_recent', ...
         'P_el','soc','esc','P_fcbus','V_bus'};
groups = struct('cand', 1:3, 'task', 4:10, 'config', 11:18, 'wind', 19:22, ...
                'state', 23:27, 'elec', 28:32);
X = [];
if nargin == 0, return; end
n = size(traj, 1);
c = ctx;
v = [c.task_type, c.e_chi, c.dh_rem, c.e_py, c.kappa, c.d_wp, c.turn_next, ...
     c.mass, c.x_cg, c.SM, c.Jy, c.P_fc, c.sc_use, c.Td, c.f_c, ...
     c.W, c.turb, c.cos_rel, c.sin_rel, ...
     c.Va, c.phi, c.gamma, c.alpha, c.u_recent, ...
     c.P_el, c.soc, c.esc, c.P_fcbus, c.V_bus];
X = [traj, repmat(v, n, 1)];
X(~isfinite(X)) = 0;
end
