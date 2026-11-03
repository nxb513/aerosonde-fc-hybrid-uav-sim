function pol = policy_def(name)
%POLICY_DEF Dinh nghia chinh sach E3 (bao_cao/thiet_ke_E3.pdf, muc 8).
%   pol = POLICY_DEF(name). Truong:
%     kind    'explore' (kieu bay lien tuc ngau nhien, thu du lieu) | 'fixed' (kieu bay co
%             dinh) | 'model' (planner dung AI)
%     model   bo mo hinh: 'full' | 'A4' (khong cau hinh) | 'A5' (khong gio) | 'A6' (khong
%             trang thai dien)
%     conf    hieu chinh conformal cua R1 va R2: 'group' (theo nhom gio) | 'global' | 'none'
%     r1, r2  co ap rang buoc R1 (AFP) / R2 (bien stall) hay khong
%     jrr     co khoan rui ro nhiem vu J_rr hay khong
%     kt, ku  he so phat thoi gian / tin hieu dieu khien (mac dinh 1)
%     delta   muc rui ro cua R2 (mac dinh 0.01)
%     obj     'J' (ham chi phi gam H2) | 'min_u' (chon can tren R1 nho nhat) | 'norm'
%             (4 khoan chia trung vi, khong don vi)
%     deg     cong chi phi suy giam quy ra H2 [R14] vao Delta M
%     ems     'filter' (loc tan so f_c co dinh) | 'adapt' (loc thich nghi [R3]) |
%             'rule' [R27] | 'ecms' [R30]
%     traj    [phi_max gamma_max Va] cho chinh sach co dinh
%   Bien the phan cung (A7 khong sieu tu) va do nhay S1 (Td), S2 (chu ky), S6 (f_c) dung
%   chinh sach 'ours'; khac biet nam o kich ban (grid_def).
pol = struct('name', name, 'kind', 'model', 'model', 'full', 'conf', 'group', ...
             'r1', true, 'r2', true, 'jrr', true, 'kt', 1, 'ku', 1, 'delta', 0.01, ...
             'obj', 'J', 'deg', false, 'ems', 'filter', 'traj', [30 6 25]);
switch name
    case 'explore',        pol.kind = 'explore';
    case {'ours', 'A7_no_sc', 'S1_Td1', 'S1_Td20', 'S2_Tp1', 'S2_Tp4', 'S6_fc006', 'S6_fc03'}
    case 'fixed_gentle',   pol.kind = 'fixed'; pol.traj = [15 2 25];
    case 'fixed_nominal',  pol.kind = 'fixed';
    case 'fixed_aggr',     pol.kind = 'fixed'; pol.traj = [45 10 25];
    case 'nominal_adapt',  pol.kind = 'fixed'; pol.ems = 'adapt';
    case 'nominal_rule',   pol.kind = 'fixed'; pol.ems = 'rule';
    case 'nominal_ecms',   pol.kind = 'fixed'; pol.ems = 'ecms';
    case 'min_risk',       pol.obj = 'min_u';
    case 'A1_no_afp',      pol.r1 = false; pol.jrr = false;
    case 'A2_no_conf',     pol.conf = 'none';
    case 'A3_global_conf', pol.conf = 'global';
    case 'A4_no_config',   pol.model = 'A4';
    case 'A5_no_wind',     pol.model = 'A5'; pol.conf = 'global';
    case 'A6_no_elec',     pol.model = 'A6';
    case 'C1_no_jrr',      pol.jrr = false;
    case 'C2_no_r2',       pol.r2 = false;
    case 'C3_no_time',     pol.kt = 0;
    case 'C4_no_ctrl',     pol.ku = 0;
    case 'C5_norm',        pol.obj = 'norm';
    case 'C6_no_r1',       pol.r1 = false;
    case 'ours_deg',       pol.deg = true;
    case 'S3_kt05',        pol.kt = 0.5;
    case 'S3_kt2',         pol.kt = 2;
    case 'S4_ku05',        pol.ku = 0.5;
    case 'S4_ku2',         pol.ku = 2;
    case 'S5_d005',        pol.delta = 0.005;
    case 'S5_d02',         pol.delta = 0.02;
    otherwise
        error('policy_def:name', 'Chinh sach khong ton tai: %s', name);
end
end
