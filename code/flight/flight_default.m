function fl = flight_default(fl)
%FLIGHT_DEFAULT Gia tri mac dinh cho mot chuyen bay vong kin E3 (EXPERIMENTS.md).
%   Nguon (power_params): P_fc, fc_wkg, endur_h, Td, ramp, Np, sc_type, esr_sc, eta_dc, arch.
%   Khoi luong/CG (hybrid_config): m_remove, dx_cg. Co cau chap hanh, stall (ac_params):
%   alpha0_deg, dmax_deg, rate_deg, delay; aero_seed bat dinh khi dong (ac_perturb).
%   W_lvl    muc gio ("mua", m/s); gio tung nhiem vu xem task_specs
%   fill     muc nap H2 ban dau (phan cua dung tich binh); khoi luong binh giu nguyen
%   soc0     SoC pin ban dau (cung la SoC tham chieu cua EMS); esc0 nang luong sieu tu
%   policy   ten chinh sach (policy_def)
%   T_plan   chu ky quyet dinh (s); H chan troi du bao / giu ung vien khi tham do (s)
%   T_max    gioi han thoi gian bay (s); dung som khi het H2 hoac SoC < soc_stop [R27]
%   seed     chuoi nhiem vu + gio (task_specs); wind_seed nhieu Dryden
%   dt       buoc tich phan 0.01 s: o gio lang, ket qua lech < 0.5 % (H2, SoC, thoi gian
%            nhiem vu) so voi 0.0025 s va cung phan loai vi pham (kiem tra 2026-11-01)
if nargin < 1, fl = struct(); end
d = struct( ...
    'P_fc', 600, 'fc_wkg', 458, 'endur_h', 1, 'Td', 10, 'ramp', 2, 'Np', 1, ...
    'sc_type', 2, 'esr_sc', 0.010, 'eta_dc', 0.95, 'arch', 'semi', ...
    'm_remove', 2.5, 'dx_cg', -0.04, ...
    'alpha0_deg', 14, 'dmax_deg', 25, 'rate_deg', 200, 'delay', 0.07, 'aero_seed', 0, ...
    'W_lvl', 4, 'h0', 100, 'fill', 1, 'soc0', 0.8, 'esc0', 0.5, 'soc_stop', 0.30, ...
    'policy', 'ours', 'T_plan', 3, 'H', 10, 'T_max', 4*3600, ...
    'seed', 1, 'wind_seed', 1, 'dt', 0.01, 'id', 0);
f = fieldnames(d);
for k = 1:numel(f)
    if ~isfield(fl, f{k}), fl.(f{k}) = d.(f{k}); end
end
end
