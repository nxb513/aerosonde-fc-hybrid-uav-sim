classdef SystemTest < matlab.unittest.TestCase
%SYSTEMTEST Kiem tra he nguon, gio Dryden, cau hinh khoi luong va bo chay kich ban.

    methods (Test)
        function drydenVariance(tc)
            % Phuong sai dau ra phai bang sigma^2 (chuan hoa trong dryden_step)
            w = dryden_init(8, 30, 100, 1, 20, 0.005, 3);
            n = 4e5; g = zeros(n, 3);
            for k = 1:n, w = dryden_step(w); g(k, :) = w.gust_b.'; end
            tc.verifyEqual(std(g), w.sigma, 'RelTol', 0.12);
        end

        function powerBalanceSemiActive(tc)
            pp = power_params(struct('P_fc', 350));
            s = ps_init(pp, 0.8, 0.5, 300);
            for k = 1:2000
                P = 300 + 200*(k > 500 && k < 1200);
                [s, o] = ps_step(pp, s, P, 0.005, @ems_filter);
                tc.assertEqual(o.P_fc + o.P_sc + o.P_bat, o.P_dem, 'AbsTol', 1e-6);
            end
            tc.verifyGreaterThan(o.soc, 0.75);
        end

        function fcRampLimitsCurrentRise(tc)
            % [R25]: FC khong the tang dong nhanh hon gioi han -> pin gánh qua do
            pp = power_params(struct('P_fc', 350, 'ramp', 2, 'use_sc', false));
            s = ps_init(pp, 0.8, 0.5, 100);
            I0 = s.I_fc; dt = 0.005;
            for k = 1:200, [s, o] = ps_step(pp, s, 400, dt, @ems_rule); end
            tc.verifyLessThanOrEqual(s.I_fc - I0, pp.fc.ramp*200*dt + 1e-9);
            tc.verifyGreaterThan(o.P_bat, 0);
        end

        function hybridConfigHitsTargetCg(tc)
            pp = power_params(struct('P_fc', 250));
            p = x8_params();
            for dx = [-0.03 0 0.02]
                comps = hybrid_config(pp, dx, 0.8);
                p1 = x8_apply_config(p, comps);
                tc.verifyEqual(p1.mass.r_cg(1), dx, 'AbsTol', 1e-12);
            end
        end

        function scenarioCalmGentleIsClean(tc)
            r = run_scenario(struct('T', 8, 'W_h', 0, 'turb', 0.01, 'phi_max', 15, ...
                'gamma_max', 3, 'Va', 22, 'dh', 10));
            tc.verifyTrue(r.ok);
            tc.verifyLessThan(r.m_cmd, 1);
            tc.verifyEqual(r.sat_pos_t, 0);
        end
    end
end
