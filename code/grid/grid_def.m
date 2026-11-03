function S = grid_def(exp)
%GRID_DEF Danh sach chuyen bay E3 (tai lap duoc), xem EXPERIMENTS.md va
%   bao_cao/thiet_ke_E3.pdf muc 8.
%   S = GRID_DEF('E3X')  2000 chuyen bay tham do 10 phut, kieu bay lien tuc ngau nhien
%                        giu 10 s (du lieu huan luyen AFP, rung bien stall, mo hinh thay
%                        the, learning curve)
%   S = GRID_DEF('E3')   danh gia: 22 chinh sach x 144 kich ban + 12 ban do nhay x 48
%                        kich ban (seed 1) = 3744 chuyen bay
%   S = GRID_DEF('E3T')  7 chuyen bay ngan de kiem tra quy trinh
%   Cau hinh "nhe": FC 600 W @ 458 W/kg [R3], binh 1 h (~13.7 kg);
%   "nang": FC 600 W @ 224 W/kg [R5], binh 2 h (~16.8 kg). Dai tham so: checks/flight_envelope.
cfg = [600 458 1; 600 224 2];
switch exp
    case 'E3X'
        N = 2000;
        rs = RandStream('mt19937ar', 'Seed', 2031);
        u = @(a, b) a + (b - a)*rand(rs);
        pick = @(v) v(randi(rs, numel(v)));
        S = repmat(flight_default(), N, 1);
        for i = 1:N
            s = flight_default();
            c = cfg(randi(rs, 2), :);
            s.P_fc = c(1); s.fc_wkg = c(2); s.endur_h = c(3);
            s.dx_cg = u(-0.08, 0); s.W_lvl = u(0, 12);
            s.soc0 = u(0.35, 0.95); s.esc0 = u(0.3, 1);
            s.Td = pick([1 10 20]); s.f_c = pick([0.06 0.1 0.3]);
            if rand(rs) < 0.2, s.sc_type = 0; end
            s.policy = 'explore'; s.T_max = 600;
            s.seed = 5000 + i; s.wind_seed = 5000 + i; s.aero_seed = 5000 + i;
            s.id = i;
            S(i) = orderfields(s, S(1));
        end
    case 'E3'
        main = {'ours', 'fixed_gentle', 'fixed_nominal', 'fixed_aggr', 'nominal_adapt', ...
                'nominal_rule', 'nominal_ecms', 'min_risk', ...
                'A1_no_afp', 'A2_no_conf', 'A3_global_conf', 'A4_no_config', 'A5_no_wind', ...
                'A6_no_elec', 'A7_no_sc', ...
                'C1_no_jrr', 'C2_no_r2', 'C3_no_time', 'C4_no_ctrl', 'C5_norm', 'C6_no_r1', ...
                'ours_deg'};
        sens = {'S1_Td1', 'S1_Td20', 'S2_Tp1', 'S2_Tp4', 'S3_kt05', 'S3_kt2', ...
                'S4_ku05', 'S4_ku2', 'S5_d005', 'S5_d02', 'S6_fc006', 'S6_fc03'};
        [a1, a2, a3, a4, a5] = ndgrid(1:2, [-0.06 -0.02], [0 4 8 12], [0.25 0.5 1], 1:3);
        base = repmat(flight_default(), numel(a1), 1);
        rs = RandStream('mt19937ar', 'Seed', 2032);
        for j = 1:numel(a1)
            s = flight_default();
            c = cfg(a1(j), :);
            s.P_fc = c(1); s.fc_wkg = c(2); s.endur_h = c(3);
            s.dx_cg = a2(j); s.W_lvl = a3(j); s.fill = a4(j);
            s.soc0 = 0.5 + 0.4*rand(rs); s.esc0 = 0.3 + 0.7*rand(rs);
            s.seed = 100*j + a5(j); s.wind_seed = s.seed; s.aero_seed = 100 + a5(j);
            base(j) = orderfields(s, base(1));
        end
        S = repmat(flight_default(), 0, 1); id = 0;
        for k = 1:numel(main)
            for j = 1:numel(base)
                s = with_policy(base(j), main{k}); id = id + 1; s.id = id;
                S(end+1, 1) = orderfields(s, base(1)); %#ok<AGROW>
            end
        end
        sub = find(a5(:) == 1).';                 % do nhay chi tren seed 1 (48 kich ban)
        for k = 1:numel(sens)
            for j = sub
                s = with_policy(base(j), sens{k}); id = id + 1; s.id = id;
                S(end+1, 1) = orderfields(s, base(1)); %#ok<AGROW>
            end
        end
    case 'E3T'
        pols = {'explore', 'fixed_gentle', 'fixed_nominal', 'fixed_aggr', 'nominal_adapt', ...
                'nominal_rule', 'nominal_ecms'};
        S = repmat(flight_default(), numel(pols), 1);
        for i = 1:numel(pols)
            s = flight_default(); s.policy = pols{i}; s.T_max = 120; s.W_lvl = 2*i;
            s.seed = 7 + i; s.wind_seed = 7 + i; s.id = i;
            S(i) = orderfields(s, S(1));
        end
    otherwise
        error('grid_def:exp', 'Thi nghiem khong ton tai: %s', exp);
end
end

function s = with_policy(s, name)
% Chinh sach + thay doi phan cung/do nhay di kem
s.policy = name;
switch name
    case 'A7_no_sc', s.sc_type = 0;
    case 'S1_Td1',   s.Td = 1;
    case 'S1_Td20',  s.Td = 20;
    case 'S2_Tp1',   s.T_plan = 1;
    case 'S2_Tp4',   s.T_plan = 4;
    case 'S6_fc006', s.f_c = 0.06;
    case 'S6_fc03',  s.f_c = 0.3;
end
end
