classdef SystemTest < matlab.unittest.TestCase
%SYSTEMTEST Kiem tra he nguon (ca gioi han cung), dong co, servo, gio Dryden va cau hinh.

    methods (Test)
        function drydenVariance(tc)
            % Phuong sai dau ra phai bang sigma^2 (chuan hoa trong dryden_step)
            w = dryden_init(8, 30, 100, 1, 20, 0.005, 3);
            n = 4e5; g = zeros(n, 3);
            for k = 1:n, w = dryden_step(w); g(k, :) = w.gust_b.'; end
            tc.verifyEqual(std(g), w.sigma, 'RelTol', 0.12);
        end

        function powerBalanceSemiActive(tc)
            % P_fc + P_sc + P_bat = P_dem - P_thieu + P_xa o moi buoc
            pp = power_params(struct('P_fc', 600));
            s = ps_init(pp, 0.8, 0.5, 400);
            for k = 1:2000
                P = 400 + 400*(k > 500 && k < 1200);
                [s, o] = ps_step(pp, s, P, 0.005, @ems_filter);
                tc.assertEqual(o.P_fc + o.P_sc + o.P_bat, o.P_dem - o.P_short + o.P_dump, 'AbsTol', 1e-6);
            end
            tc.verifyGreaterThan(o.soc, 0.75);
        end

        function fcRampLimitsCurrentRise(tc)
            % [R25]: FC khong the tang dong nhanh hon gioi han -> pin ganh qua do
            pp = power_params(struct('P_fc', 600, 'ramp', 2, 'sc_type', 0));
            s = ps_init(pp, 0.8, 0.5, 150);
            I0 = s.I_fc; dt = 0.005;
            for k = 1:200, [s, o] = ps_step(pp, s, 700, dt, @ems_rule); end
            tc.verifyLessThanOrEqual(s.I_fc - I0, pp.fc.ramp*200*dt + 1e-9);
            tc.verifyGreaterThan(o.P_bat, 0);
        end

        function fcPowerNeverAboveRated(tc)
            % Tai lon keo dai: cong suat stack khong bao gio vuot dinh muc 600 W. EMS loc
            % tan so ra lenh vuot 600 W nen gioi han cung phai tac dong; cac EMS con lai
            % tu gioi han lenh, gioi han cung chi la chot chan.
            pp = power_params(struct('P_fc', 600, 'sc_type', 0));
            for ems = {@ems_filter, @ems_rule, @ems_ecms, @ems_adapt}
                s = ps_init(pp, 0.6, 0.5, 400);
                Pmax = 0;
                for k = 1:6000
                    [s, o] = ps_step(pp, s, 1100, 0.01, ems{1});
                    Pmax = max(Pmax, o.P_stack);
                end
                tc.verifyLessThanOrEqual(Pmax, pp.fc.P_rated + 1e-9);
                if isequal(ems{1}, @ems_filter), tc.verifyGreaterThan(s.lim.fc_pwr, 0); end
            end
        end

        function batteryDischargeLimited(tc)
            % Khong sieu tu, tai 1500 W: dong pin <= 10C; phan thieu bao ra, tran dong co
            pp = power_params(struct('P_fc', 600, 'sc_type', 0));
            s = ps_init(pp, 0.5, 0.5, 400);
            Imax = 0; Pshort = 0;
            for k = 1:3000
                [s, o] = ps_step(pp, s, 400 + 1100*(k > 200), 0.01, @ems_filter);
                Imax = max(Imax, o.I_bat); Pshort = max(Pshort, o.P_short);
            end
            tc.verifyLessThanOrEqual(Imax, pp.bat.I_dis_max + 1e-9);
            tc.verifyGreaterThan(Pshort, 0);
            tc.verifyLessThan(o.P_motor_cap, 1500);
        end

        function batteryChargeLimitedAndStopsWhenFull(tc)
            % Tai nhe, SoC tham chieu dat cao hon 1 (chi trong test) de EMS ep nap manh:
            % dong nap <= 1C, SoC khong vuot 1, gioi han nap va ngat khi day deu tac dong
            pp = power_params(struct('P_fc', 600, 'sc_type', 0));
            s = ps_init(pp, 0.97, 0.5, 100); s.soc0 = 1.3;
            Imin = 0; socmax = 0;
            for k = 1:60000
                [s, o] = ps_step(pp, s, 30, 0.01, @ems_filter);
                Imin = min(Imin, o.I_bat); socmax = max(socmax, o.soc);
            end
            tc.verifyGreaterThanOrEqual(Imin, -pp.bat.I_chg_max - 1e-9);
            tc.verifyLessThanOrEqual(socmax, 1 + 1e-12);
            tc.verifyGreaterThan(s.lim.bat_chg, 0);
            tc.verifyGreaterThan(s.lim.bat_full, 0);
        end

        function supercapStaysWithinLimits(tc)
            % Tai dao dong manh: nang luong sieu tu trong [25 %, 100 %], dong <= Imax
            pp = power_params(struct('P_fc', 600));
            s = ps_init(pp, 0.8, 0.3, 400);
            e = zeros(8000, 1); I = e;
            for k = 1:8000
                P = 400 + 700*(mod(floor(k/150), 2) == 1);
                [s, o] = ps_step(pp, s, P, 0.01, @ems_filter);
                e(k) = o.esc; I(k) = o.I_sc;
            end
            tc.verifyGreaterThanOrEqual(min(e), pp.sc.e_min - 1e-9);
            tc.verifyLessThanOrEqual(max(e), 1 + 1e-9);
            tc.verifyLessThanOrEqual(max(abs(I)), pp.sc.Imax + 1e-9);
        end

        function motorVoltageCapped(tc)
            % Bus 47.2 V (pin day), ga 100 %: dien ap vao dong co bi kep o 44.4 V [R37]
            p = ac_params();
            o = ac_propulsion(25, 1, 47.2, p);
            tc.verifyEqual(o.Vin, p.prop.Vmax, 'AbsTol', 1e-12);
            o2 = ac_propulsion(25, 1, 47.2, p, 30);
            tc.verifyEqual(o2.Vin, 30, 'AbsTol', 1e-12);
            tc.verifyLessThan(o2.Pel, o.Pel);
        end

        function actuatorLimitsWithFlightIntegrator(tc)
            % Tich phan dung nhu run_flight (RK4 0.01 s, kep sau moi buoc): lenh vuot gioi
            % han goc va toc do -> canh lai thuc khong bao gio vuot 25 deg va 200 deg/s
            p = ac_params(); a = p.act; dt = 0.01; d2r = pi/180;
            nS = round(a.delay/dt);
            cmds = {@(t) 60*d2r*(t >= 0.5), @(t) 40*d2r*sin(2*pi*6*t)};
            for c = 1:2
                x = zeros(6, 1); buf = zeros(3, nS); D = zeros(400, 3);
                for k = 1:400
                    buf = [buf(:, 2:end), cmds{c}(k*dt)*[1; -1; 1]];
                    f = @(xx) servo(xx, [buf(:, 1); 0.5], p);
                    k1 = f(x); k2 = f(x + 0.5*dt*k1); k3 = f(x + 0.5*dt*k2); k4 = f(x + dt*k3);
                    x = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
                    x(1:3) = min(max(x(1:3), -a.max), a.max);
                    x(4:6) = min(max(x(4:6), -a.rate), a.rate);
                    D(k, :) = x(1:3).';
                end
                tc.verifyLessThanOrEqual(max(abs(D(:))), a.max(1) + 1e-12);
                tc.verifyLessThanOrEqual(max(max(abs(diff(D))))/dt, a.rate(1)*(1 + 1e-9));
            end
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

        function drydenRetuneScalesOutput(tc)
            % Doi cuong do giua chuyen bay: sigma moi, bo loc giu nguyen
            w = dryden_init(8, 30, 100, 1, 25, 0.01, 3);
            w2 = dryden_retune(w, 8, 100, 2);
            tc.verifyEqual(w2.sigma, 2*w.sigma, 'RelTol', 1e-12);
            tc.verifyEqual(w2.sys{3}.C, 2*w.sys{3}.C, 'RelTol', 1e-12);
            tc.verifyEqual(w2.sys{3}.Ad, w.sys{3}.Ad);
        end

        function emsFilterCutoff(tc)
            % f_c thap -> FC muot hon (tong bien thien cong suat nho hon)
            dP = zeros(1, 2); fcs = [0.06 1];
            for j = 1:2
                pp = power_params(struct('P_fc', 600, 'f_c', fcs(j)));
                s = ps_init(pp, 0.8, 0.5, 400);
                for k = 1:2000
                    P = 400 + 300*(mod(floor(k/200), 2) == 1);
                    s = ps_step(pp, s, P, 0.005, @ems_filter);
                end
                dP(j) = s.fc_dP;
            end
            tc.verifyLessThan(dP(1), dP(2));
        end
    end
end

function dx = servo(x, ca, p)
[d_dot, dd_dot] = ac_actuator_deriv(x(1:3), x(4:6), 0.5, ca, p);
dx = [d_dot; dd_dot];
end
