classdef AircraftTest < matlab.unittest.TestCase
%AIRCRAFTTEST Kiem tra tinh hop ly vat ly cua mo hinh Aerosonde [R37, App. E].
%   [R37] khong cong bo diem trim/cuc so cho Aerosonde, nen kiem tra dua tren tinh
%   chat vat ly (can bang nang luong, xu huong theo khoi luong/CG, dau cac mode).

    properties
        p
        d2r = pi/180
    end

    methods (TestMethodSetup)
        function load(tc)
            tc.p = ac_params();
        end
    end

    methods (Test)
        function trimPlausible(tc)
            tr = ac_trim(tc.p, 25, 0, struct('Ub', 42.9));
            tc.verifyTrue(tr.converged);
            tc.verifyGreaterThan(tr.alpha/tc.d2r, 1); tc.verifyLessThan(tr.alpha/tc.d2r, 6);
            tc.verifyGreaterThan(tr.dt, 0.3); tc.verifyLessThan(tr.dt, 1);
            tc.verifyGreaterThan(tr.Pel, 250); tc.verifyLessThan(tr.Pel, 600);
            % Luc can cua mo hinh khi dong ~ luc day (bay ngang)
            tc.verifyEqual(tr.T*cos(tr.alpha), ...
                0.5*tc.p.env.rho*25^2*tc.p.geom.S*(tc.p.aero.CDp + (tc.p.aero.CL0 + tc.p.aero.CLa*tr.alpha)^2/(pi*tc.p.geom.e*tc.p.geom.AR) + tc.p.aero.CDde*abs(tr.de)), ...
                'RelTol', 0.25);
        end

        function propulsionEfficiencyBelowOne(tc)
            for Va = [15 20 25 30]
                for d = [0.85 0.95 1]
                    o = ac_propulsion(Va, d, 42.9, tc.p);
                    if o.Pel > 0, tc.verifyLessThan(o.T*Va, o.Pel); end
                end
            end
        end

        function linearModesPlausible(tc)
            lin = ac_linearize(tc.p, ac_trim(tc.p, 25, 0, struct('Ub', 42.9)));
            el = lin.eig_lon; ex = lin.eig_lat;
            sp = el(abs(imag(el)) > 2); ph = el(abs(imag(el)) <= 2 & abs(imag(el)) > 0);
            tc.verifyTrue(all(real(sp) < 0) && all(real(ph) < 0));
            tc.verifyGreaterThan(abs(sp(1)), abs(ph(1)));
            dr = ex(abs(imag(ex)) > 1);
            tc.verifyTrue(all(real(dr) < 0));
            tc.verifyLessThan(min(real(ex)), -5);       % mode lan nhanh
        end

        function trimHoldsOpenLoop(tc)
            tr = ac_trim(tc.p, 24, 0, struct('Ub', 42.9));
            I = ac_state_index(); x = tr.x0; dt = 0.005;
            for k = 1:1000
                f = @(xx) ac_derivatives(xx, tr.cmd, 42.9, zeros(3,1), zeros(3,1), tc.p);
                k1 = f(x); k2 = f(x + dt/2*k1); k3 = f(x + dt/2*k2); k4 = f(x + dt*k3);
                x = x + dt/6*(k1 + 2*k2 + 2*k3 + k4); x(I.q) = x(I.q)/norm(x(I.q));
            end
            tc.verifyLessThan(abs(norm(x(I.vel)) - 24), 0.01);
        end

        function actuatorLimits(tc)
            d = zeros(3, 1); dd = zeros(3, 1); dt = 0.001; a = tc.p.act;
            for k = 1:600
                [d1, d2] = ac_actuator_deriv(d, dd, 0.5, [1; -1; 1; 0.5], tc.p);
                d = min(max(d + dt*d1, -a.max), a.max); dd = min(max(dd + dt*d2, -a.rate), a.rate);
                tc.assertLessThanOrEqual(abs(d), a.max + 1e-12);
            end
            tc.verifyEqual(abs(d), a.max, 'AbsTol', 1e-3);
        end

        function massPropertiesParallelAxis(tc)
            c = struct('name', 'x', 'm', 1.0, 'r', [0.2; 0; 0.05], 'J', []);
            p1 = ac_apply_config(tc.p, c);
            mu = 11*1/12;
            tc.verifyEqual(p1.mass.J(2,2) - tc.p.mass.J(2,2), mu*(0.2^2 + 0.05^2), 'AbsTol', 1e-12);
            p2 = ac_apply_config(tc.p, [c, struct('name', 'bo', 'm', -1.0, 'r', [0.2; 0; 0.05], 'J', [])]);
            tc.verifyEqual(p2.mass.J, tc.p.mass.J, 'AbsTol', 1e-12);
        end

        function forwardCgNeedsMoreUpElevator(tc)
            t0 = ac_trim(tc.p, 22, 0, struct('Ub', 42.9));
            p1 = tc.p; p1.mass.r_cg = [0.03; 0; 0];
            t1 = ac_trim(p1, 22, 0, struct('Ub', 42.9));
            tc.verifyLessThan(t1.de, t0.de);
            tc.verifyLessThan(t1.margin, t0.margin);
        end

        function heavierNeedsMoreAlphaAndPower(tc)
            t0 = ac_trim(tc.p, 24, 0, struct('Ub', 42.9));
            p1 = ac_apply_config(tc.p, struct('name', 'tai', 'm', 3, 'r', [0;0;0], 'J', []));
            t1 = ac_trim(p1, 24, 0, struct('Ub', 42.9));
            tc.verifyGreaterThan(t1.alpha, t0.alpha);
            tc.verifyGreaterThan(t1.Pel, t0.Pel);
        end

        function lowerBusVoltageNeedsMoreThrottle(tc)
            t1 = ac_trim(tc.p, 24, 0, struct('Ub', 46));
            t2 = ac_trim(tc.p, 24, 0, struct('Ub', 41));
            tc.verifyGreaterThan(t2.dt, t1.dt);
        end

        function quaternionRoundTrip(tc)
            e = [0.3; -0.2; 2.5];
            tc.verifyEqual(quat_utils('to_euler', quat_utils('from_euler', e)), e, 'AbsTol', 1e-12);
        end
    end
end
