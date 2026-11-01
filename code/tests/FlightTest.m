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

        function packedForestMatchesTreeBagger(tc)
            rng(3); X = rand(600, 5); y = X(:, 1) + 0.1*randn(600, 1);
            B = TreeBagger(20, X, y, 'Method', 'regression', 'MinLeafSize', 5);
            P = rf_pack(B, X, y);
            Xq = rand(50, 5);
            tc.verifyEqual(rf_predict(P, Xq), predict(B, Xq), 'AbsTol', 1e-12);
            q = qrf_predict(P, Xq, 0.9);
            tc.verifyGreaterThan(mean(q >= rf_predict(P, Xq)), 0.95);
        end
    end
end
