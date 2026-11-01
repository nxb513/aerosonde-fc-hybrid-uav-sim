function ub = cqr_bound(C, q, grp)
%CQR_BOUND Can tren da hieu chuan: q + c (toan cuc) hoac q + c_nhom.
if nargin < 3 || isempty(C.by_group)
    ub = q + C.global;
    return
end
ub = q + C.global;
for k = 1:numel(C.groups)
    ub(grp == C.groups(k)) = q(grp == C.groups(k)) + C.by_group(k);
end
end
