function S = grid_def(exp)
%GRID_DEF Danh sach kich ban cua thi nghiem (xac dinh, tai lap duoc).
%   S = GRID_DEF('E1')  lay mau ngau nhien 10000 kich ban cho du lieu AFP
%   S = GRID_DEF('E2')  luoi co cau truc 20736 kich ban (cau hinh x gio x
%                       nhiem vu x ung vien x seed) cho danh gia quyet dinh
%   S = GRID_DEF('T')   luoi nho de thu (8 kich ban)
%   Moi phan tu la struct dung cho run_scenario. Xem EXPERIMENTS.md.

switch exp
    case 'E1'
        N = 10000;
        rs = RandStream('mt19937ar', 'Seed', 2026);
        u = @(a, b) a + (b - a)*rand(rs);
        pick = @(v) v(randi(rs, numel(v)));
        S = repmat(scenario_default(), N, 1);
        for i = 1:N
            s = scenario_default();
            s.P_fc = pick([250 350 450]); s.Np = pick([1 2]); s.use_sc = pick([false true]);
            s.Td = pick([1 10 20]);
            s.m_remove = u(0.5, 1.0); s.dx_cg = u(-0.04, 0.04);
            s.aero_seed = i;
            s.W_h = u(0, 12); s.wind_dir = u(0, 360); s.turb = u(0.5, 2.5); s.wind_seed = i;
            s.dchi = pick([45 90 135 180]); s.dh = u(0, 60);
            s.phi_max = u(15, 45); s.gamma_max = u(2, 10); s.Va = u(17, 25);
            s.id = i;
            S(i) = orderfields(s, S(1));
        end
    case 'E2'
        cfgP = [250 450]; cfgX = [-0.025 0 0.025];
        Wh = [0 4 8 12]; turb = [1 2];
        dchi = [90 180]; dh = [0 40];
        phi = [15 25 35 45]; gam = [3 6 9]; Va = [18 21 24];
        seeds = 1:3;
        [a1,a2,a3,a4,a5,a6,a7,a8,a9,a10] = ndgrid(cfgP, cfgX, Wh, turb, dchi, dh, phi, gam, Va, seeds);
        N = numel(a1);
        S = repmat(scenario_default(), N, 1);
        rs = RandStream('mt19937ar', 'Seed', 7);
        dirs = 360*rand(rs, 1000, 1);
        for i = 1:N
            s = scenario_default();
            s.P_fc = a1(i); s.dx_cg = a2(i); s.m_remove = 1.0; s.Np = 1; s.use_sc = true; s.Td = 10;
            s.W_h = a3(i); s.turb = a4(i); s.dchi = a5(i); s.dh = a6(i);
            s.phi_max = a7(i); s.gamma_max = a8(i); s.Va = a9(i);
            sd = a10(i);
            % Cung seed -> cung gio/khi dong cho moi ung vien (so sanh cap doi)
            key = sum([find(cfgP == a1(i)), find(cfgX == a2(i)), find(Wh == a3(i)), ...
                       find(turb == a4(i)), find(dchi == a5(i)), find(dh == a6(i))].*[1 2 6 24 48 96]);
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
