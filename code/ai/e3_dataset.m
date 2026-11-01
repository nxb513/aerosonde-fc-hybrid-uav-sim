function D = e3_dataset(traces)
%E3_DATASET Bang mau (moi cua so quyet dinh mot dong) tu cac chuyen bay tham do E3X.
%   D = E3_DATASET(traces), traces lay tu gather_results('E3X').
%   Cot cua windows (run_flight): [id, nhiem vu, 33 dac trung (e3_features), dur, m,
%   m_cmd, m_alpha, m_rate, tien do, H2, P pin+sieu tu, suy giam pin, bien thien P_FC,
%   P dinh, roi]. Bo cua so ngan hon 2 s (bi cat boi ket thuc nhiem vu/chuyen bay).
%   Nhan hoi quy la toc do (chia cho dur) nen so sanh duoc giua cac do dai cua so.
W = {};
for k = 1:numel(traces)
    t = traces{k};
    if isstruct(t) && isfield(t, 'windows') && ~isempty(t.windows), W{end+1} = double(t.windows); end %#ok<AGROW>
end
W = vertcat(W{:});
W = W(W(:, 36) >= 2, :);
[~, D.names, D.groups] = e3_features();
D.flight = W(:, 1); D.task = W(:, 2); D.X = W(:, 3:35);
lab = {'dur','m','m_cmd','m_alpha','m_rate','prog','h2','eb','bd','fd','ppk','crash'};
for j = 1:numel(lab), D.(lab{j}) = W(:, 35 + j); end
D.crash = D.crash > 0;
D.viol = D.m > 1;
D.wbin = discretize(D.X(:, strcmp(D.names, 'W')), [-inf 3 7 10 inf]);
end
