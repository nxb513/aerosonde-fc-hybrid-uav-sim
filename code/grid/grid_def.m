function S = grid_def(exp)
%GRID_DEF Danh sach kich ban thi nghiem (tai lap duoc) cho Aerosonde-FC.
%   S = GRID_DEF('E1')  10000 kich ban lay mau ngau nhien (du lieu huan luyen AFP)
%   S = GRID_DEF('E2')  20736 kich ban luoi co cau truc (kiem thu + danh gia quyet dinh)
%   S = GRID_DEF('T')   8 kich ban de thu
%   Co so chon dai tham so: checks/flight_envelope (results/flight_envelope.md) va
%   EXPERIMENTS.md. Moi struct dung truc tiep cho run_scenario.
switch exp
    case 'E1'
        N = 10000;
        rs = RandStream('mt19937ar', 'Seed', 2027);
        u = @(a, b) a + (b - a)*rand(rs);
        pick = @(v) v(randi(rs, numel(v)));
        S = repmat(scenario_default(), N, 1);
        for i = 1:N
            s = scenario_default();
            s.P_fc = pick([400 600 800]); s.fc_wkg = pick([224 458]); s.endur_h = pick([1 2]);
            s.Np = pick([1 2]); s.sc_type = pick([0 1 2]); s.Td = pick([1 10 20]); s.ramp = pick([2 5]);
            s.m_remove = u(1.5, 3.0); s.dx_cg = u(-0.08, 0.0);
            s.alpha0_deg = u(12, 16); s.dmax_deg = u(20, 30); s.rate_deg = u(100, 250);
            s.delay = u(0.03, 0.08); s.aero_seed = i;
            s.W_h = u(0, 12); s.wind_dir = u(0, 360); s.turb = u(0.5, 3); s.wind_seed = i;
            s.dchi = pick([45 90 135 180]); s.dh = u(0, 60);
            s.phi_max = u(15, 50); s.gamma_max = u(1, 10); s.Va = u(19, 30);
            s.id = i;
            S(i) = orderfields(s, S(1));
        end
    case 'E2'
        % Cau hinh 1 "nhe": FC 600 W @ 458 W/kg, binh 1 h (~13.7 kg)
        % Cau hinh 2 "nang": FC 600 W @ 224 W/kg, binh 2 h (~16.8 kg)
        cfg = [600 458 1; 600 224 2];
        cfgI = 1:2; cgX = [-0.06 -0.04 -0.02];
        Wh = [0 4 8 12]; turb = [1 2.5];
        dchi = [90 180]; dh = [0 50];
        phi = [15 25 35 45]; gam = [2 5 8]; Va = [22 25 28];
        seeds = 1:3;
        [a1,a2,a3,a4,a5,a6,a7,a8,a9,a10] = ndgrid(cfgI, cgX, Wh, turb, dchi, dh, phi, gam, Va, seeds);
        N = numel(a1);
        S = repmat(scenario_default(), N, 1);
        rs = RandStream('mt19937ar', 'Seed', 11);
        dirs = 360*rand(rs, 1000, 1);
        for i = 1:N
            s = scenario_default();
            s.P_fc = cfg(a1(i), 1); s.fc_wkg = cfg(a1(i), 2); s.endur_h = cfg(a1(i), 3);
            s.dx_cg = a2(i); s.W_h = a3(i); s.turb = a4(i); s.dchi = a5(i); s.dh = a6(i);
            s.phi_max = a7(i); s.gamma_max = a8(i); s.Va = a9(i);
            sd = a10(i);
            % Cung tinh huong + seed -> cung gio/khi dong cho moi ung vien (so sanh cap doi)
            key = sum([a1(i), find(cgX == a2(i)), find(Wh == a3(i)), find(turb == a4(i)), ...
                       find(dchi == a5(i)), find(dh == a6(i))].*[1 2 6 24 48 96]);
            s.wind_seed = 1000*sd + key; s.aero_seed = 500 + sd;
            s.wind_dir = dirs(mod(s.wind_seed, 1000) + 1);
            s.id = i;
            S(i) = orderfields(s, S(1));
        end
    case 'T'
        S = repmat(scenario_default(), 8, 1);
        for i = 1:8
            s = scenario_default(); s.T = 20; s.phi_max = 15 + 4*i; s.W_h = i; s.id = i;
            S(i) = orderfields(s, S(1));
        end
    otherwise
        error('grid_def:exp', 'Thi nghiem khong ton tai: %s', exp);
end
end
