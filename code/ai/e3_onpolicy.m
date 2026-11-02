function Tab = e3_onpolicy(T, traces, H)
%E3_ONPOLICY Kiem tra AFP tren quy dao cua chinh sach (chan troi H giay).
%   Tab = E3_ONPOLICY(T, traces, H). Dung trace.onpolicy cua run_flight: sau moi quyet
%   dinh, m thuc te tren H giay tiep theo (planner van quyet dinh lai) so voi can tren ub
%   va luong tu tho q cua ung vien da chon. Chi tinh cua so du H giay (khong bi cat do
%   chuyen bay dung). Moi chinh sach 'model' mot dong:
%     cov, cov_g1..4   ti le m <= ub (toan bo, theo nhom gio)
%     viol, viol_ok    ti le m > 1; ti le m > 1 khi ub <= 1
%     cov_recal        do phu khi hieu chuan lai bang du lieu cua chinh sach: c_g tinh tren
%                      cac chuyen bay seed 1-2 (CQR mot phia [R32]), kiem tra tren seed 3
%                      (tach theo chuyen bay de tranh ro ri)
%   Neu khong co trace.onpolicy (ban cu) thi tra ve bang rong.
if nargin < 3, H = 10; end
eps_c = 0.1;
pols = unique(T.in_policy(~ismissing(T.in_policy)), 'stable');
rows = {};
for pname = pols.'
    sel = find(T.in_policy == pname);
    OP = []; rep = [];
    for j = sel.'
        t = traces{j};
        if isstruct(t) && isfield(t, 'onpolicy') && ~isempty(t.onpolicy)
            o = double(t.onpolicy);
            OP = [OP; o]; rep = [rep; repmat(mod(T.in_seed(j), 100), size(o, 1), 1)]; %#ok<AGROW>
        end
    end
    if isempty(OP), continue; end
    full = OP(:, 13) >= H - 1e-6;
    OP = OP(full, :); rep = rep(full);
    ub = OP(:, 6); q = OP(:, 7); g = OP(:, 8); m = OP(:, 9);
    r = struct('policy', pname, 'n', numel(m), 'cov', mean(m <= ub));
    for k = 1:4, r.(sprintf('cov_g%d', k)) = mean(m(g == k) <= ub(g == k)); end
    r.viol = mean(m > 1); r.viol_ok = mean(m(ub <= 1) > 1); r.frac_ub_le1 = mean(ub <= 1);
    cal = rep <= 2; te = rep == 3;
    if any(cal) && any(te)
        C = cqr_calibrate(q(cal), m(cal), eps_c, g(cal));
        u2 = cqr_bound(C, q(te), g(te));
        r.cov_recal = mean(m(te) <= u2);
        r.cov_test_orig = mean(m(te) <= ub(te));
    else
        r.cov_recal = NaN; r.cov_test_orig = NaN;
    end
    rows{end+1} = r; %#ok<AGROW>
end
if isempty(rows), Tab = table(); else, Tab = struct2table([rows{:}]'); end
end
