classdef X8ModelTest < matlab.unittest.TestCase
%X8MODELTEST Kiem chung mo hinh X8 voi so lieu cong bo va cac tinh chat vat ly.
%   Chay: run_tests (trong thu muc code).

    properties
        p
        d2r = pi/180
    end

    methods (TestMethodSetup)
        function load(tc)
            tc.p = x8_params();
        end
    end

    methods (Test)
        function linearizationMatchesR2(tc)
            % Ma tran A va cuc tai diem hoat dong cua [R2, Eq. 36-42, Bang 10-11].
            % Sai khac con lai: rho va Omega_p tai diem do [R2] khong cong bo.
            [op, ref] = r2_reference_point(tc.p);
            lin = x8_linearize(tc.p, op, struct('g_b', op.g_b));
            tc.verifyLessThan(max(abs(lin.A_lon(:) - ref.A_lon(:))), 0.25);
            tc.verifyLessThan(max(abs(lin.A_lat(:) - ref.A_lat(:))), 0.9);
            sp = lin.eig_lon(abs(imag(lin.eig_lon)) > 2);
            tc.verifyLessThan(abs(abs(sp(1)) - abs(ref.eig_lon(1)))/abs(ref.eig_lon(1)), 0.05);
            ph = lin.eig_lon(abs(imag(lin.eig_lon)) < 2);
            tc.verifyLessThan(abs(abs(ph(1)) - abs(ref.eig_lon(3)))/abs(ref.eig_lon(3)), 0.05);
            dr = lin.eig_lat(abs(imag(lin.eig_lat)) > 1);
            tc.verifyLessThan(abs(abs(dr(1)) - abs(ref.eig_lat(2)))/abs(ref.eig_lat(2)), 0.05);
            tc.verifyLessThan(abs(min(real(lin.eig_lat)) - ref.eig_lat(1))/abs(ref.eig_lat(1)), 0.06);
        end

        function trimMatchesR2(tc)
            % [R2, Eq. 36-38]: Va 17.9, alpha 7.9, theta -1.11 deg -> gamma_a = -9.01 deg,
            % de -2.35 deg, dt 0.44. Diem cua [R2] lay tu maneuver, khong phai can bang chinh xac.
            tr = x8_trim(tc.p, 17.9, -9.01*tc.d2r);
            tc.verifyTrue(tr.converged);
            tc.verifyLessThan(abs(tr.alpha/tc.d2r - 7.9), 0.5);
            tc.verifyLessThan(abs(tr.de/tc.d2r + 2.35), 0.5);
            tc.verifyLessThan(abs(tr.dt - 0.44), 0.03);
        end

        function trimHoldsInSimulation(tc)
            tr = x8_trim(tc.p, 18, 0);
            out = x8_simulate(tc.p, tr.x0, 5, 0.002, tr.cmd);
            tc.verifyLessThan(max(abs(out.Va - 18)), 1e-3);
            tc.verifyLessThan(max(abs(out.alpha - tr.alpha)), 1e-4);
            tc.verifyFalse(any(out.sat_pos | out.sat_rate));
        end

        function elevonLimits(tc)
            tr = x8_trim(tc.p, 18, 0);
            cmd = tr.cmd; cmd(1:2) = 45*tc.d2r;
            out = x8_simulate(tc.p, tr.x0, 0.6, 0.001, cmd, struct('cmd0', tr.cmd));
            I = x8_state_index();
            e = tc.p.act.elevon;
            tc.verifyLessThanOrEqual(max(abs(out.x(:, I.dlt)), [], 'all'), e.max + 1e-12);
            tc.verifyLessThanOrEqual(max(abs(out.x(:, I.ddlt)), [], 'all'), e.rate + 1e-12);
            tc.verifyTrue(any(out.sat_rate));
            tc.verifyTrue(all(out.sat_pos(2:end)));
            % Tre 0.07 s [R2, Bang 9]: elevon chua dich chuyen truoc 0.07 s
            k = find(out.t < 0.069, 1, 'last');
            tc.verifyLessThan(max(abs(out.x(1:k, I.dlt(1)) - tr.der)), 1e-9);
        end

        function massPropertiesParallelAxis(tc)
            p0 = tc.p;
            c = struct('name', 'x', 'm', 0.5, 'r', [0.1; 0; 0.05], 'J', []);
            p1 = x8_apply_config(p0, c);
            tc.verifyEqual(p1.mass.m, 3.864, 'AbsTol', 1e-12);
            tc.verifyEqual(p1.mass.r_cg, 0.5*[0.1; 0; 0.05]/3.864, 'AbsTol', 1e-12);
            % Hai chat diem: Iy tang mu*d^2, mu = m0*m/(m0+m), d^2 = 0.1^2 + 0.05^2
            mu = 3.364*0.5/3.864;
            tc.verifyEqual(p1.mass.J(2,2) - p0.mass.J(2,2), mu*(0.1^2 + 0.05^2), 'AbsTol', 1e-12);
            c2 = [c, struct('name', 'bo', 'm', -0.5, 'r', [0.1; 0; 0.05], 'J', [])];
            p2 = x8_apply_config(p0, c2);
            tc.verifyEqual(p2.mass.J, p0.mass.J, 'AbsTol', 1e-12);
            tc.verifyEqual(p2.mass.r_cg, zeros(3, 1), 'AbsTol', 1e-12);
        end

        function forwardCgNeedsMoreUpElevon(tc)
            tr0 = x8_trim(tc.p, 18, 0);
            p1 = x8_apply_config(tc.p, struct('name', 'can', 'm', 0.3, 'r', [0.15; 0; 0], 'J', []));
            tr1 = x8_trim(p1, 18, 0);
            tc.verifyGreaterThan(p1.mass.r_cg(1), 0);
            tc.verifyLessThan(tr1.de, tr0.de);
            tc.verifyLessThan(tr1.elevon_margin, tr0.elevon_margin);
        end

        function heavierNeedsMoreAlphaAndPower(tc)
            tr0 = x8_trim(tc.p, 18, 0);
            p1 = x8_apply_config(tc.p, struct('name', 'tai', 'm', 1.0, 'r', [0; 0; 0], 'J', []));
            tr1 = x8_trim(p1, 18, 0);
            tc.verifyGreaterThan(tr1.alpha, tr0.alpha);
            tc.verifyGreaterThan(tr1.Pel, tr0.Pel);
        end

        function lowerBusVoltageNeedsMoreThrottle(tc)
            % Diem ghep voi he nguon: cung luc day, Ub thap -> ga cao hon [R2, muc 2.3.5]
            tr1 = x8_trim(tc.p, 18, 0, struct('Ub', 16.4));
            tr2 = x8_trim(tc.p, 18, 0, struct('Ub', 15.2));
            tc.verifyGreaterThan(tr2.dt, tr1.dt);
            tc.verifyEqual(tr2.Pel, tr1.Pel, 'RelTol', 1e-6);
        end

        function ds1ValidationManeuvers(tc)
            % Mo phong ho vong 2 maneuver kiem chung cua [DS1] (khong dung de nhan dang).
            % Nguong la muc dat duoc luc tao test (2026-11-01), dung de phat hien hoi quy.
            root = fileparts(fileparts(mfilename('fullpath')));
            ddir = fullfile(root, '..', 'data', 'DS1', 'validation');
            tc.assumeTrue(isfolder(ddir), 'Chua tai du lieu DS1 vao data/DS1');
            addpath(fullfile(root, 'validation'));
            r = ds1_simulate_maneuver(tc.p, ds1_load(fullfile(ddir, 'longitudinal_3211_5.mat')), 0.0025);
            c = corrcoef(r.sim.alpha, r.meas.alpha);
            tc.verifyGreaterThan(c(1, 2), 0.9);
            c = corrcoef(r.sim.q, r.meas.q);
            tc.verifyGreaterThan(c(1, 2), 0.9);
            tc.verifyLessThan(abs(mean(r.sim.Ibus) - mean(r.meas.Ibus))/mean(r.meas.Ibus), 0.15);
            r = ds1_simulate_maneuver(tc.p, ds1_load(fullfile(ddir, 'lateral_121_4.mat')), 0.0025);
            c = corrcoef(r.sim.p, r.meas.p);
            tc.verifyGreaterThan(c(1, 2), 0.9);
            c = corrcoef(r.sim.r, r.meas.r);
            tc.verifyGreaterThan(c(1, 2), 0.8);
        end

        function quaternionRoundTrip(tc)
            e = [0.3; -0.2; 2.5];
            tc.verifyEqual(quat_utils('to_euler', quat_utils('from_euler', e)), e, 'AbsTol', 1e-12);
            R = quat_utils('dcm', quat_utils('from_euler', e));
            tc.verifyEqual(R.'*R, eye(3), 'AbsTol', 1e-12);
        end
    end
end
