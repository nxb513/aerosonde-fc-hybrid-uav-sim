function node = rf_leaf(P, X)
%RF_LEAF Nut la cua moi dong X trong moi cay cua rung dong goi (rf_pack).
%   node = RF_LEAF(P, X): ma tran nr x T chi so nut (toan cuc). Quy tac chia nhanh cua
%   cay hoi quy MATLAB: x < CutPoint -> nhanh trai.
nr = size(X, 1);
node = repmat(P.root, nr, 1);
rowi = repmat((1:nr).', 1, numel(P.root));
act = find(P.left(node) > 0);
while ~isempty(act)
    nd = node(act);
    xv = X(rowi(act) + (double(P.feat(nd)) - 1)*nr);
    nx = P.right(nd);
    gl = xv < P.cut(nd);
    nx(gl) = P.left(nd(gl));
    node(act) = nx;
    act = act(P.left(nx) > 0);
end
end
