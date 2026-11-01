function q = qrf_predict(P, X, tau)
%QRF_PREDICT Luong tu tau theo kieu rung hoi quy luong tu, tren rung dong goi (rf_pack).
%   q = QRF_PREDICT(P, X, tau). Trong so cua mau huan luyen i:
%   w_i(x) = (1/T) sum_t 1{i trong la_t(x)} / |la_t(x)|; q = luong tu co trong so cua y.
%   Day la cach TreeBagger/quantilePredict uoc luong phan bo co dieu kien; ban nay
%   vector hoa de du nhanh cho planner. Do phu cuoi cung do CQR dam bao [R32].
node = rf_leaf(P, X);
[nr, T] = size(node);
q = zeros(nr, 1);
for i = 1:nr
    nd = node(i, :).';
    st = P.ls(nd); ct = P.lc(nd); tot = sum(ct);
    ids = repelem(st, ct) + (0:tot - 1).' - repelem(cumsum(ct) - ct, ct);
    w = repelem(1./(T*ct), ct);
    [ys, o] = sort(P.ly(ids));
    k = find(cumsum(w(o)) >= tau - 1e-12, 1);
    q(i) = double(ys(k));
end
end
