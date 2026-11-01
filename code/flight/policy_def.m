function pol = policy_def(name)
%POLICY_DEF Dinh nghia chinh sach lap lich cho chuyen bay E3.
%   pol = POLICY_DEF(name). Truong:
%     kind    'explore' (ung vien ngau nhien, thu du lieu) | 'model' (AFP + mo hinh thay the)
%             | 'fixed' (quy dao co dinh)
%     model   bo mo hinh: 'full' | 'A3' (khong dac trung cau hinh) | 'A4' (khong gio)
%             | 'A11' (khong trang thai dien)
%     conf    rang buoc AFP: 'group' (conformal theo nhom gio) | 'global' | 'none' (luong
%             tu tho) | 'off' (khong rang buoc)
%     lam     [] = lambda theo nhiem vu, so = lambda co dinh
%     mode    'both' (chon quy dao + muc bu) | 'replan' (chi doi quy dao, FC theo tai,
%             cam dua vao pin cho dinh cong suat) | 'comp' (giu quy dao danh nghia, chi
%             chon muc bu)
%     traj    [phi_max gamma_max Va] cho chinh sach co dinh
%     ems     EMS cho chinh sach co dinh: 'filter' ([R3], f_c thich nghi) | 'rule' [R27]
%             | 'ecms' [R30]
%   Cac bien the phan cung (A5 khong sieu tu, A9 thu dong) va do nhay (S1 Td, S2 chu ky
%   planner) dung chinh sach 'ours'; khac biet nam o kich ban (grid_def).
pol = struct('name', name, 'kind', 'model', 'model', 'full', 'conf', 'group', ...
             'lam', [], 'mode', 'both', 'traj', [35 5 25], 'ems', 'filter');
switch name
    case 'explore',          pol.kind = 'explore';
    case {'ours', 'A5_no_sc', 'A9_passive', 'S1_Td1', 'S1_Td20', 'S2_Tp1', 'S2_Tp5'}
    case 'A1_no_afp',        pol.conf = 'off';
    case 'A2_no_conf',       pol.conf = 'none';
    case 'A3_no_config',     pol.model = 'A3';
    case 'A4_no_wind',       pol.model = 'A4';
    case 'A6_replan_only',   pol.mode = 'replan';
    case 'A7_comp_only',     pol.mode = 'comp';
    case 'A8_fixed_lambda',  pol.lam = 0.5;
    case 'A10_global_conf',  pol.conf = 'global';
    case 'A11_no_elec',      pol.model = 'A11';
    case 'fixed_gentle',     pol.kind = 'fixed'; pol.traj = [15 2 25];
    case 'fixed_nominal',    pol.kind = 'fixed';
    case 'fixed_aggr',       pol.kind = 'fixed'; pol.traj = [45 8 25];
    case 'nominal_rule',     pol.kind = 'fixed'; pol.ems = 'rule';
    case 'nominal_ecms',     pol.kind = 'fixed'; pol.ems = 'ecms';
    otherwise
        error('policy_def:name', 'Chinh sach khong ton tai: %s', name);
end
end
