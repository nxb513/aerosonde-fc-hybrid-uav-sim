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
            pp = power_params(struct('P_fc', 600));
            s = ps_init(pp, 0.8, 0.5, 400);
            for k = 1:2000
                P = 400 + 400*(k > 500 && k < 1200);
                [s, o] = ps_step(pp, s, P, 0.005, @ems_filter);
                tc.assertEqual(o.P_fc + o.P_sc + o.P_bat, o.P_dem, 'AbsTol', 1e-6);
            end
            tc.verifyGreaterThan(o.soc, 0.75);
        end

        function fcRampLimitsCurrentRise(tc)
            % [R25]: FC khong the tang dong nhanh hon gioi han -> pin gánh qua do
            pp = power_params(struct('P_fc', 600, 'ramp', 2, 'sc_type', 0));
            s = ps_init(pp, 0.8, 0.5, 150);
            I0 = s.I_fc; dt = 0.005;
            for k = 1:200, [s, o] = ps_step(pp, s, 700, dt, @ems_rule); end
            tc.verifyLessThanOrEqual(s.I_fc - I0, pp.fc.ramp*200*dt + 1e-9);
            tc.verifyGreaterThan(o.P_bat, 0);
        end

        function hybridConfigHitsTargetCg(tc)
            pp = power_params(struct('P_fc', 600));
            p = ac_params();
            for dx = [-0.06 -0.02 0]
                comps = hybrid_config(pp, dx, 2.5, p.mass.m0);
                p1 = ac_apply_config(p, comps);
                tc.verifyEqual(p1.mass.r_cg(1), dx, 'AbsTol', 1e-12);
            end
        end

        function scenarioCalmGentleIsClean(tc)
            r = run_scenario(struct('T', 10, 'W_h', 0, 'turb', 0.01, 'phi_max', 15, ...
                'gamma_max', 2, 'Va', 24, 'dh', 10));
            tc.verifyTrue(r.ok);
            tc.verifyLessThan(r.m_cmd, 1);
            tc.verifyEqual(r.sat_pos_t, 0);
        end
    end
end
