function [R, choice] = decision_eval(T2, models, S, lambdas)
%DECISION_EVAL Danh gia chinh sach chon quy dao (bu nang luong <-> doi quy dao) tren E2.
%   [R, choice] = DECISION_EVAL(T2, models, S, lambdas)
%   Moi "tinh huong" = (P_fc, dx_cg, gio, nhieu, nhiem vu, seed) co 36 ung vien.
%   Chi phi thuc te (sau mo phong):
%     J = lambda*tau + (1 - lambda)*(e + d)  neu khong vi pham va hoan thanh
%     J = 10                                 neu vi pham (m > 1) hoac khong hoan thanh
%     tau = t_done/20 s, e = E_el/(300 W * 40 s), d = 0.5*(dmg_pin/trung vi + dP_FC/trung vi)
%   Chinh sach chi dung thong tin truoc chuyen bay (dac trung afp_dataset):
%     oracle       chon theo chi phi thuc te (can duoi)
%     ours         surrogate J + rang buoc AFP conformal (can tren <= 1)
%     A1_no_afp    surrogate J, bo rang buoc co cau chap hanh
%     A2_no_conf   rang buoc bang luong tu tho (khong hieu chuan)
%     A3_no_config / A4_no_wind  AFP thieu nhom dac trung
%     A6_replan_only  chi xet ung vien co P_peak du bao <= P_fc (khong dua vao pin/sieu tu)
%     A7_comp_only    giu quy dao danh nghia (phi 35, gamma 6, Va 22), de he nguon bu
%     A8_fixed_lambda chinh sach dung lambda = 0.5 bat ke lambda that
%     always_gentle / always_aggr  quy dao co dinh (15/3/22) va (45/9/22)
if nargin < 4, lambdas = [0.2 0.5 0.8]; end
D = afp_dataset(T2);
ok = D.ok & T2.completed & ~D.viol;
med_dmg = median(T2.bat_dmg(ok)); med_dp = median(T2.fc_dP(ok));
tau = T2.t_done/20; e = T2.E_el/(300*40);
d = 0.5*(T2.bat_dmg/med_dmg + T2.fc_dP/med_dp);

% Du bao truoc chuyen bay
pc = predict(S.complete, D.X); th = predict(S.t_done, D.X)/20;
eh = predict(S.E_el, D.X)/(300*40); ph = predict(S.P_peak, D.X);
dh = 0.5*(predict(S.bat_dmg, D.X)/med_dmg + predict(S.fc_dP, D.X)/med_dp);
ub = struct();
for v = {'full', 'A2_no_conf', 'A3_no_config', 'A4_no_wind'}
    m = models.(v{1});
    q = afp_predict(m.M, D.X);
    if strcmp(m.mode, 'none'), ub.(v{1}) = q; else, ub.(v{1}) = cqr_bound(m.C, q); end
end

key = [T2.in_P_fc, T2.in_dx_cg, T2.in_W_h, T2.in_turb, T2.in_dchi, T2.in_dh, T2.in_aero_seed];
[~, ~, sit] = unique(key, 'rows');
nsit = max(sit);
pol = {'oracle','ours','A1_no_afp','A2_no_conf','A3_no_config','A4_no_wind', ...
       'A6_replan_only','A7_comp_only','A8_fixed_lambda','always_gentle','always_aggr'};
fixed = @(ph_, g_, v_) T2.in_phi_max == ph_ & T2.in_gamma_max == g_ & T2.in_Va == v_;
isnom = fixed(35, 6, 22); isgen = fixed(15, 3, 22); isagg = fixed(45, 9, 22);
R = table(); choice = struct();
for L = lambdas
    Jr = L*tau + (1 - L)*(e + d); Jr(~ok) = 10;
    Jh = L*th + (1 - L)*(eh + dh); Jh(pc < 0.5) = 10;
    Jh5 = 0.5*th + 0.5*(eh + dh); Jh5(pc < 0.5) = 10;
    ch = zeros(nsit, numel(pol));
    for s = 1:nsit
        r = find(sit == s);
        ch(s, 1) = pick(r, Jr(r), true(size(r)), Jr(r));
        ch(s, 2) = pick(r, Jh(r), ub.full(r) <= 1, ub.full(r));
        ch(s, 3) = pick(r, Jh(r), true(size(r)), Jh(r));
        ch(s, 4) = pick(r, Jh(r), ub.A2_no_conf(r) <= 1, ub.A2_no_conf(r));
        ch(s, 5) = pick(r, Jh(r), ub.A3_no_config(r) <= 1, ub.A3_no_config(r));
        ch(s, 6) = pick(r, Jh(r), ub.A4_no_wind(r) <= 1, ub.A4_no_wind(r));
        ch(s, 7) = pick(r, Jh(r), ub.full(r) <= 1 & ph(r) <= T2.in_P_fc(r), ph(r));
        ch(s, 8) = r(find(isnom(r), 1));
        ch(s, 9) = pick(r, Jh5(r), ub.full(r) <= 1, ub.full(r));
        ch(s, 10) = r(find(isgen(r), 1));
        ch(s, 11) = r(find(isagg(r), 1));
    end
    Jo = Jr(ch(:, 1));
    for k = 1:numel(pol)
        c = ch(:, k);
        R = [R; table(L, string(pol{k}), mean(~ok(c)), mean(D.viol(c)), mean(Jr(c)), ...
            mean(Jr(c) - Jo), mean(T2.t_done(c(ok(c)))), mean(T2.E_el(c))/1e3, ...
            mean(T2.bat_dmg(c)), mean(T2.fc_dP(c)), mean(T2.P_peak(c)), ...
            'VariableNames', {'lambda','policy','fail_rate','viol_rate','J_mean','regret', ...
            't_done_ok','E_kJ','bat_dmg','fc_dP','P_peak'})]; %#ok<AGROW>
    end
    choice.(sprintf('L%02d', round(10*L))) = array2table(ch, 'VariableNames', pol);
end
end

function c = pick(r, J, feas, fallback)
% Chon argmin J trong tap kha thi; neu rong chon argmin cua tieu chi du phong
if any(feas)
    rr = r(feas); [~, k] = min(J(feas)); c = rr(k);
else
    [~, k] = min(fallback); c = r(k);
end
end
