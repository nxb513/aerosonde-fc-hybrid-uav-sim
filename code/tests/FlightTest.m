classdef FlightTest < matlab.unittest.TestCase
%FLIGHTTEST Kiem tra chuyen bay vong kin E3: nhiem vu, bam duong, dieu kien dung,
%   checkpoint, muc vi pham m, planner (rang buoc, thu tu noi), luoi thi nghiem, rung.

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

        function tasksHaveNoDeadline(tc)
            % Nhiem vu khong co han; chong ket theo kieu bay danh nghia (30, 6, 25)
            tk = task_make(struct('type', 3, 'u', [0.5 0.9 0 0]), [0; 0], 0, 100);
            tc.verifyFalse(isfield(tk, 'deadline'));
            tc.verifyEqual(tk.L, abs(tk.h_t - 100)/sind(6), 'RelTol', 1e-12);
            tc.verifyEqual(tk.T_abort, max(3*tk.T_dn, tk.T_dn + 120), 'RelTol', 1e-12);
        end

        function stopsWhenHydrogenRunsOut(tc)
            fl = struct('policy', 'fixed_nominal', 'fill', 0.001, 'W_lvl', 2, 'id', 1);
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

        function violationIndexHasFiveParts(tc)
            % Moi cua so tham do ghi 5 thanh phan; m = max(5 thanh phan) (khi khong roi)
            r = run_flight(struct('policy', 'explore', 'T_max', 60, 'W_lvl', 8, 'id', 2), []);
            W = double(r.trace.windows); n = r.trace.window_names;
            c = @(s) W(:, strcmp(n, s));
            mc = [c('m1') c('m2') c('m3') c('m4') c('m5')];
            ok = c('crash') == 0;
            tc.verifyEqual(c('m'), max(mc, [], 2), 'AbsTol', 1e-5);
            tc.verifyTrue(all(ok));
            tc.verifyTrue(all(c('u2') >= 0 & c('u2') <= 1));
            tc.verifyTrue(all(c('loff') >= 0 & c('loff') <= 40));   % m/s, khong vuot van toc dat
            Xp = W(:, 3:5);                                    % kieu bay lien tuc trong luoi
            tc.verifyTrue(all(Xp(:, 1) >= 10 & Xp(:, 1) <= 50 & Xp(:, 2) >= 2 & Xp(:, 2) <= 10));
        end

        function onPolicyWindowPerDecision(tc)
            % Chinh sach 'model': moi quyet dinh mot cua so H giay tren quy dao chinh sach
            M = tiny_models();
            r = run_flight(struct('policy', 'ours', 'T_max', 30, 'W_lvl', 4, 'id', 5), M);
            OP = double(r.trace.onpolicy); DL = double(r.trace.decisions);
            no = r.trace.onpolicy_names; nd = r.trace.decision_names;
            tc.verifyEqual(size(OP, 1), r.n_dec);
            tc.verifyEqual(OP(:, strcmp(no, 'u')), DL(:, strcmp(nd, 'u')), 'AbsTol', 1e-6);
            full = OP(:, strcmp(no, 'dur')) >= 10 - 1e-6;
            tc.verifyTrue(all(OP(full, 1) <= 20 + 1e-6));        % cua so du 10 s bat dau truoc 20 s
            tc.verifyTrue(all(OP(~full, 1) > 20 - 1e-6));        % cua so bi cat khi dung
            tc.verifyEqual(r.n_lvl0 + r.n_lvl1 + r.n_lvl2, r.n_dec);
        end

        function plannerConstraintsAndFallback(tc)
            % Mo hinh tong hop: m = phi/40 (R1 loai phi > 40), a = (30 - Va)/6 (R2 loai
            % Va < 24), loff = phi/50. Kieu bay chon phai dat R1, R2; min_risk chon u nho
            % nhat; chi cho Va 22 -> khong dat R2 -> chon a nho nhat (muc 2).
            M = synth_models();
            C = e3_candidates();
            ctx = synth_ctx(C.Va);
            [tj, ~, info] = planner_decide(policy_def('ours'), M, ctx, C, RandStream('mt19937ar'));
            tc.verifyLessThanOrEqual(info.u, 1); tc.verifyLessThanOrEqual(info.a, 1);
            tc.verifyEqual(info.level, 0); tc.verifyGreaterThanOrEqual(tj(3), 24);
            [~, ~, i2] = planner_decide(policy_def('min_risk'), M, ctx, C, RandStream('mt19937ar'));
            tc.verifyLessThanOrEqual(i2.u, info.u + 1e-9);
            ctx.Va_allowed = 22;
            [~, ~, i3] = planner_decide(policy_def('ours'), M, ctx, C, RandStream('mt19937ar'));
            tc.verifyEqual(i3.level, 2);
        end

        function gridMatchesDesign(tc)
            S = grid_def('E3');
            tc.verifyEqual(numel(S), (22 + 12)*48);
            pols = unique({S.policy});
            tc.verifyEqual(numel(pols), 34);
            for k = 1:numel(pols)
                policy_def(pols{k});                            % ten nao sai se bao loi
                s = S(strcmp({S.policy}, pols{k}));
                tc.verifyEqual(numel(unique([s.seed])), 48);    % moi chinh sach du 48 kich ban
            end
            tc.verifyTrue(all([S.fill] == 1) && all([S.m_H2] == 0.1) && all([S.fc_wkg] == 458));
            X = grid_def('E3X');
            tc.verifyEqual(numel(X), 2000);
            tc.verifyTrue(all(ismember([X.f_c], [0.06 0.1 0.3])));
            C = e3_candidates();
            tc.verifyEqual(size(C.traj, 1), 315);
            tc.verifyEqual(C.traj(C.nominal, :), [30 6 25]);
        end

        function packedForestMatchesTreeBagger(tc)
            rng(3); X = rand(600, 5); y = X(:, 1) + 0.1*randn(600, 1);
            B = TreeBagger(20, X, y, 'Method', 'regression', 'MinLeafSize', 5);
            P = rf_pack(B, X, y);
            Xq = rand(50, 5);
            % Gia tri la luu dang single -> sai so lam tron ~1e-8, duong di trong cay giong het
            tc.verifyEqual(rf_predict(P, Xq), predict(B, Xq), 'RelTol', 1e-6);
            [q, pe] = qrf_predict(P, Xq, 0.9, 0.5);
            tc.verifyGreaterThan(mean(q >= rf_predict(P, Xq)), 0.95);
            tc.verifyTrue(all(pe >= 0 & pe <= 1 + 1e-12));
            tc.verifyGreaterThan(corr(pe, Xq(:, 1)), 0.8);       % P(y > 0.5) tang theo x1
        end
    end
end

function M = tiny_models()
% Bo mo hinh nho tren du lieu ngau nhien, chi de chay planner trong test
rng(1); n = 400;
[~, names] = e3_features();
cols = 1:numel(names);
X = rand(n, numel(names)); y = rand(n, 1)*1.2;
fit = @(yy, q) packf(TreeBagger(5, X, yy, 'Method', 'regression', 'MinLeafSize', 20), X, yy, q, cols);
grp = randi(4, n, 1);
mdl.afp = fit(y, true);
mdl.C = cqr_calibrate(qrf_predict(mdl.afp, X, 0.9), y, 0.1, grp);
mdl.stall.P = fit(y, true);
for dl = [0.005 0.01 0.02]
    mdl.stall.(sprintf('d%04d', round(dl*1e4))) = struct('C', ...
        cqr_calibrate(qrf_predict(mdl.stall.P, X, 1 - dl), y, dl, grp), 'use_group', false, 'n_group', []);
end
for s = {'prog', 'h2', 'pb', 'pbat', 'loff', 'fcuv', 'bd'}, mdl.(s{1}) = fit(y, false); end
M.full = mdl;
M.norm = struct('H', 10, 'V_ref', 25, 'LHV', 120e6, 's0', 2.5, 'k_soc', 3, 'soc_floor', 0.35, ...
                'c_H2', 6.9, 'med', [1 1 1 1]);
end

function P = packf(B, X, y, q, cols)
if q, P = rf_pack(B, X, y); else, P = rf_pack(B); end
P.cols = cols;
end

function M = synth_models()
% Rung tren du lieu tong hop co quy luat ro (xem plannerConstraintsAndFallback)
rng(2); n = 6000;
[~, names] = e3_features();
cols = 1:numel(names);
X = rand(n, numel(names));
X(:, 1) = 10 + 40*rand(n, 1); X(:, 2) = 2 + 8*rand(n, 1); X(:, 3) = 22 + 6*rand(n, 1);
m = X(:, 1)/40; a = (30 - X(:, 3))/6;
B = @(yy) TreeBagger(30, X, yy, 'Method', 'regression', 'MinLeafSize', 5);
P = @(yy) setfield(rf_pack(B(yy), X, yy), 'cols', cols); %#ok<SFLD>
R = @(yy) setfield(rf_pack(B(yy)), 'cols', cols); %#ok<SFLD>
mdl.afp = P(m);
mdl.C = struct('alpha', 0.1, 'global', 0, 'by_group', [], 'groups', []);
mdl.stall.P = P(a);
mdl.stall.d0100 = struct('C', struct('alpha', 0.01, 'global', 0, 'by_group', [], 'groups', []), ...
                         'use_group', false, 'n_group', []);
mdl.prog = R(X(:, 3)); mdl.h2 = R(1e-5*ones(n, 1)); mdl.pb = R(zeros(n, 1));
mdl.pbat = R(zeros(n, 1)); mdl.loff = R(X(:, 1)/50);
M.full = mdl;
M.norm = struct('H', 10, 'V_ref', 25, 'LHV', 120e6, 's0', 2.5, 'k_soc', 3, 'soc_floor', 0.35, ...
                'c_H2', 6.9, 'med', [1 1 1 1]);
end

function ctx = synth_ctx(Va)
[~, names] = e3_features();
for k = 4:numel(names), ctx.(names{k}) = 0.5; end
ctx.task_type = 1; ctx.W = 4;
ctx.soc_ref = 0.5; ctx.E_bat_J = 3.5e5; ctx.Va_allowed = Va; ctx.Va_range = [22 28];
ctx.mdot_ht = 8e-6; ctx.f_ht = 8e-6/25; ctx.L_nv = 2000; ctx.task_violated = false;
ctx.dV_eol_uV = 7e4; ctx.C_fc = 336; ctx.C_bat = 71;
end
