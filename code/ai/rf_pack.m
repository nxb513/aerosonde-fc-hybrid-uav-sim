function P = rf_pack(B, Xtr, ytr)
%RF_PACK Dong goi rung TreeBagger thanh mang de du bao nhanh (vector hoa).
%   P = RF_PACK(B, Xtr, ytr). Neu co Xtr, ytr: luu them phan bo dap ung cua du lieu
%   huan luyen trong tung la de tinh luong tu kieu rung hoi quy luong tu (qrf_predict).
%   Ly do: quantilePredict cua TreeBagger mat ~1.7 s cho 36 dong (do tren may),
%   qua cham cho planner chay moi 3 s; ban dong goi cho cung du bao trung binh voi
%   predict (kiem tra trong FlightTest) va khong can Statistics Toolbox khi bay.
T = B.NumTrees;
names = B.PredictorNames;
F = cell(T, 1); Cu = F; Le = F; Ri = F; Va = F;
root = zeros(1, T); off = 0;
for t = 1:T
    tr = B.Trees{t};
    ch = tr.Children; nn = size(ch, 1);
    br = ch(:, 1) > 0;
    f = zeros(nn, 1);
    [~, f(br)] = ismember(tr.CutPredictor(br), names);
    l = ch(:, 1); r = ch(:, 2);
    l(br) = l(br) + off; r(br) = r(br) + off;
    F{t} = f; Cu{t} = tr.CutPoint; Le{t} = l; Ri{t} = r; Va{t} = tr.NodeMean;
    root(t) = off + 1; off = off + nn;
end
P.feat = int32(vertcat(F{:})); P.cut = vertcat(Cu{:});
P.left = int32(vertcat(Le{:})); P.right = int32(vertcat(Ri{:}));
P.val = single(vertcat(Va{:})); P.root = root;
if nargin >= 3
    node = rf_leaf(P, Xtr);
    [nd, o] = sort(node(:));
    yy = repmat(ytr(:), T, 1);
    P.ly = single(yy(o));
    P.lc = accumarray(nd, 1, [numel(P.val) 1]);
    P.ls = cumsum(P.lc) - P.lc + 1;
end
end
