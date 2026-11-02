classdef FlightTest < matlab.unittest.TestCase
%FLIGHTTEST Kiem tra chuyen bay vong kin E3: nhiem vu, bam duong, dieu kien dung,
%   checkpoint giua chuyen bay va rung dong goi.

    methods (Test)
        function taskSequenceIsReproducible(tc)
            a = task_specs(4, 8); b = task_specs(4, 8);
            tc.verifyEqual([a.type], [b.type]);
            tc.verifyEqual([a.W], [b.W]);
            tc.verifyGreaterThanOrEqual(min([a.W]), 0);
            tc.verifyEqual(numel(unique([a.type])), 6);
        end

        function lineFollowingConverges(tc)
            % Diem dong hoc bay voi huong lenh: sai lech ngang giam ve 0 [R37, Eq. 10.9]
            tk = task_make(struct('type', 6, 'u', [0 0 0 0]), [0; 0], 0, 100);
            p = [0; 80]; chi = 0; V = 25; dt = 0.05;
            for k = 1:1200
                g = guidance_step(tk, p, chi, 100, V, chi);
                chi = chi + max(min(mod(g.chi_c - chi + pi, 2*pi) - pi, 0.3*dt), -0.3*dt);
                p = p + V*dt*[cos(chi); sin(chi)];
                if g.done, break; end
            end
            tc.verifyLessThan(abs(g.e_py), 2);
        end

        function orbitStartsTangent(tc)
            tk = task_make(struct('type', 4, 'u', [0.5 0.9 0 0]), [0; 0], 0.3, 100);
            g = guidance_step(tk, [0; 0], 0.3, 100, 25, 0.3);
            tc.verifyLessThan(abs(mod(g.chi_c - 0.3 + pi, 2*pi) - pi), 1e-9);
            tc.verifyGreaterThan(g.phi_ff, 0);
        end

        function stopsWhenHydrogenRunsOut(tc)
            fl = struct('policy', 'fixed_nominal', 'fill', 0.004, 'W_lvl', 2, 'id', 1);
            r = run_flight(fl, []);
            tc.verifyEqual(r.stop_reason, 'h2');
            tc.verifyGreaterThanOrEqual(r.H2_used_g, r.H2_avail_g);
            tc.verifyLessThan(r.t_flight, 120);
        end

        function midFlightCheckpointResumes(tc)
            f = [tempname '.mat'];
            fl = struct('policy', 'fixed_aggr', 'T_max', 40, 'W_lvl', 6, 'id', 3);
            r0 = run_flight(fl, []);
            [~, F] = run_flight(setfield(fl, 'T_max', 20), []); %#ok<SFLD>
            F.stop = false; F.fl.T_max = 40; save(f, 'F');
            r1 = run_flight(fl, [], struct('ckpt_file', f));
            tc.verifyEqual(r1.H2_used_g, r0.H2_used_g, 'RelTol', 1e-9);
            tc.verifyEqual(r1.soc_end, r0.soc_end, 'RelTol', 1e-9);
            tc.verifyFalse(isfile(f));
        end

        function onPolicyWindowPerDecision(tc)
            % Chinh sach 'model': moi quyet dinh mot cua so H giay tren quy dao chinh sach
            M = tiny_models();
            r = run_flight(struct('policy', 'ours', 'T_max', 40, 'W_lvl', 4, 'id', 5), M);
            OP = double(r.trace.onpolicy); DL = double(r.trace.decisions);
            tc.verifyEqual(size(OP, 1), r.n_dec);
            tc.verifyEqual(OP(:, 6), DL(:, 6), 'AbsTol', 1e-6);          % cung can tren ub
            full = OP(:, 13) >= 10 - 1e-6;
            tc.verifyTrue(all(OP(full, 1) <= 30 + 1e-6));                % cua so du 10 s bat dau truoc 30 s
            tc.verifyTrue(all(OP(~full, 1) > 30 - 1e-6));                % cua so bi cat khi dung
            tc.verifyGreaterThanOrEqual(min(OP(:, 9)), 0);
        end

        function packedForestMatchesTreeBagger(tc)
            rng(3); X = rand(600, 5); y = X(:, 1) + 0.1*randn(600, 1);
            B = TreeBagger(20, X, y, 'Method', 'regression', 'MinLeafSize', 5);
            P = rf_pack(B, X, y);
            Xq = rand(50, 5);
            % Gia tri la luu dang single -> sai so lam tron ~1e-8, duong di trong cay giong het
            tc.verifyEqual(rf_predict(P, Xq), predict(B, Xq), 'RelTol', 1e-6);
            q = qrf_predict(P, Xq, 0.9);
            tc.verifyGreaterThan(mean(q >= rf_predict(P, Xq)), 0.95);
        end
    end
end

function M = tiny_models()
% Bo mo hinh nho tren du lieu ngau nhien, chi de chay planner trong test
rng(1); n = 400;
[~, names, G] = e3_features();
cols = 1:numel(names); colsT = setdiff(cols, G.cand(4));
X = rand(n, numel(names)); y = rand(n, 1);
fit = @(c) TreeBagger(5, X(:, c), y, 'Method', 'regression', 'MinLeafSize', 20);
afp = rf_pack(fit(colsT), X(:, colsT), y); afp.cols = colsT;
regT = rf_pack(fit(colsT)); regT.cols = colsT;
regA = rf_pack(fit(cols)); regA.cols = cols;
C = cqr_calibrate(qrf_predict(afp, X(:, colsT), 0.9), y, 0.1, randi(4, n, 1));
M.full = struct('tau', 0.9, 'afp', afp, 'C', C, 'prog', regT, 'ppk', regT, ...
                'h2', regA, 'eb', regA, 'dg', regA);
M.norm = struct('H', 10, 'V_ref', 25, 'LHV', 120e6, 's0', 2.5, 'k_soc', 3, ...
                'soc_floor', 0.35, 'h2_ref', 1, 'med_bd', 1, 'med_fd', 1);
end
