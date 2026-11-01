function row = afp_metrics(q, y, C, grp, tau)
%AFP_METRICS Chi so cua AFP tren mot tap: do phu cua can tren conformal (toan cuc, theo
%   nhom), pinball cua luong tu tho, recall / bao dong gia khi canh bao "ub > 1", AUC.
%   AUC = thong ke Mann-Whitney cua q giua mau vi pham (y > 1) va khong vi pham.
ug = cqr_bound(C, q); un = cqr_bound(C, q, grp);
v = y > 1;
row.n = numel(y); row.viol_rate = mean(v);
row.cov_global = mean(y <= ug); row.cov_group = mean(y <= un);
for k = 1:4, row.(sprintf('cov_group_w%d', k)) = mean(y(grp == k) <= un(grp == k)); end
row.pinball = mean(max(tau*(y - q), (tau - 1)*(y - q)));
row.recall = mean(un(v) > 1); row.false_alarm = mean(un(~v) > 1);
r = tiedrank(q);
np = nnz(v); nn = nnz(~v);
row.auc = (sum(r(v)) - np*(np + 1)/2)/max(np*nn, 1);
end
