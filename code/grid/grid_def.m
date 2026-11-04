function S = grid_def(exp)
%GRID_DEF Danh sach chuyen bay E3 (tai lap duoc), xem EXPERIMENTS.md va
%   bao_cao/thiet_ke_E3.pdf muc 8.
%   S = GRID_DEF('E3X')  2000 chuyen bay tham do 10 phut, kieu bay lien tuc ngau nhien
%                        giu 10 s (du lieu huan luyen AFP, rung bien stall, mo hinh thay
%                        the, learning curve)
%   S = GRID_DEF('E3')   danh gia: (22 chinh sach + 12 ban do nhay) x 48 kich ban
%                        = 1632 chuyen bay; kich ban = CG {-6, -2} cm x muc gio
%                        {0, 4, 8, 12} m/s x 6 seed
%   S = GRID_DEF('E3T')  7 chuyen bay ngan de kiem tra quy trinh
%   Mot cau hinh: FC 600 W @ 458 W/kg [R3], 100 g H2 nap day, binh nen 6 % + dieu ap
%   175 g [R35] (power_params). Dai tham so: checks/flight_envelope.
switch exp
    case 'E3X'
        N = 2000;
        rs = RandStream('mt19937ar', 'Seed', 2031);
        u = @(a, b) a + (b - a)*rand(rs);
        pick = @(v) v(randi(rs, numel(v)));
        S = repmat(flight_default(), N, 1);
        for i = 1:N
            s = flight_default();
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
                'C1_no_jrr', 'C2_no_r2', 'C3_no_time', 'C4_no_track', 'C5_norm', 'C6_no_r1', ...
                'ours_deg'};
        sens = {'S1_Td1', 'S1_Td20', 'S2_Tp1', 'S2_Tp4', 'S3_kt05', 'S3_kt2', ...
                'S4_kb05', 'S4_kb2', 'S5_d005', 'S5_d02', 'S6_fc006', 'S6_fc03'};
        [a2, a3, a5] = ndgrid([-0.06 -0.02], [0 4 8 12], 1:6);
        base = repmat(flight_default(), numel(a2), 1);
        rs = RandStream('mt19937ar', 'Seed', 2032);
        for j = 1:numel(a2)
            s = flight_default();
            s.dx_cg = a2(j); s.W_lvl = a3(j);
            s.soc0 = 0.5 + 0.4*rand(rs); s.esc0 = 0.3 + 0.7*rand(rs);
            s.seed = 100*j + a5(j); s.wind_seed = s.seed; s.aero_seed = 100 + a5(j);
            base(j) = orderfields(s, base(1));
        end
        S = repmat(flight_default(), 0, 1); id = 0;
        for k = 1:numel(main) + numel(sens)   % ban do nhay chay tren ca 48 kich ban
            if k <= numel(main), name = main{k}; else, name = sens{k - numel(main)}; end
            for j = 1:numel(base)
                s = with_policy(base(j), name); id = id + 1; s.id = id;
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
