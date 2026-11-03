function [q, pexc] = qrf_predict(P, X, tau, thr)
%QRF_PREDICT Luong tu tau cua rung hoi quy luong tu [R39], tren rung dong goi (rf_pack).
%   [q, pexc] = QRF_PREDICT(P, X, tau, thr). Trong so cua mau huan luyen i [R39, Eq. 4-5]:
%   w_i(x) = (1/T) sum_t 1{i trong la_t(x)} / |la_t(x)|;
%   F(y|x) = sum_i w_i(x) 1{y_i <= y} [R39, Eq. 6]; q = inf{y : F(y|x) >= tau} [R39, Eq. 1].
%   pexc (tuy chon) = 1 - F(thr|x) = sum_i w_i(x) 1{y_i > thr}: xac suat vuot nguong thr,
%   doc tu cung phan phoi co dieu kien.
%   Ban nay vector hoa de du nhanh cho planner. QRF chi nhat quan tiem can [R39, Dinh ly 1];
%   do phu voi mau huu han do CQR dam bao [R32].
node = rf_leaf(P, X);
[nr, T] = size(node);
q = zeros(nr, 1); pexc = nan(nr, 1);
want = nargin >= 4 && nargout >= 2;
for i = 1:nr
    nd = node(i, :).';
    st = P.ls(nd); ct = P.lc(nd); tot = sum(ct);
    ids = repelem(st, ct) + (0:tot - 1).' - repelem(cumsum(ct) - ct, ct);
    w = repelem(1./(T*ct), ct);
    [ys, o] = sort(P.ly(ids));
    wo = w(o);
    k = find(cumsum(wo) >= tau - 1e-12, 1);
    q(i) = double(ys(k));
    if want, pexc(i) = sum(wo(ys > thr)); end
end
end
