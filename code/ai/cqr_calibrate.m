function C = cqr_calibrate(q_cal, y_cal, alpha, grp)
%CQR_CALIBRATE Hieu chuan conformal mot phia cho can tren (CQR, [R32]).
%   C = CQR_CALIBRATE(q_cal, y_cal, alpha, grp)
%   Diem phu hop s_i = y_i - q(x_i); hieu chinh = luong tu thu ceil((n+1)(1-alpha))
%   cua s [R32]. Bao dam P(y <= q(x) + c) >= 1 - alpha voi du lieu trao doi duoc.
%   grp (tuy chon): hieu chuan rieng tung nhom (group-balanced [R32]).
s = y_cal - q_cal;
C.alpha = alpha;
C.global = conformal_quantile(s, alpha);
C.by_group = [];
if nargin >= 4 && ~isempty(grp)
    g = unique(grp(:)).';
    C.groups = g;
    C.by_group = arrayfun(@(k) conformal_quantile(s(grp == k), alpha), g);
end
end

function c = conformal_quantile(s, alpha)
n = numel(s);
k = ceil((n + 1)*(1 - alpha));
if k > n, c = inf; return; end
ss = sort(s);
c = ss(k);
end
