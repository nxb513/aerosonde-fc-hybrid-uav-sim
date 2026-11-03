function D = e3_dataset(traces)
%E3_DATASET Bang mau (moi cua so 10 s mot dong) tu cac chuyen bay tham do E3X.
%   D = E3_DATASET(traces), traces lay tu gather_results('E3X'). Cot cua windows theo
%   trace.window_names (run_flight): id, nhiem vu, 32 dac trung (e3_features), roi nhan:
%     dur, m, m1..m5 (5 thanh phan cua m), a = alpha_p99/alpha0, prog (toc do tien, m/s),
%     h2 (kg/s), pb (cong suat pin + sieu tu ra bus, W), pbat (chi pin, W), u2 (muc dung
%     canh lai), bd (suy giam pin, %/s), fcuv (sut ap FC, uV/cell/s), crash,
%   va thong tin phu cho ham chi phi: mdot_ht (kg/s), soc_ref, L_nv (m), task_viol.
%   Bo cua so ngan hon 2 s (bi cat boi ket thuc nhiem vu/chuyen bay).
W = {}; names = {};
for k = 1:numel(traces)
    t = traces{k};
    if isstruct(t) && isfield(t, 'windows') && ~isempty(t.windows)
        W{end+1} = double(t.windows); names = t.window_names; %#ok<AGROW>
    end
end
W = vertcat(W{:});
col = @(n) W(:, strcmp(names, n));
W = W(col('dur') >= 2, :);
col = @(n) W(:, strcmp(names, n));
[~, D.names, D.groups] = e3_features();
D.flight = col('id'); D.task = col('itask');
D.X = W(:, 3:2 + numel(D.names));
lab = {'dur','m','m1','m2','m3','m4','m5','a','prog','h2','pb','pbat','u2','bd','fcuv', ...
       'crash','mdot_ht','soc_ref','L_nv','task_viol'};
for j = 1:numel(lab), D.(lab{j}) = col(lab{j}); end
D.crash = D.crash > 0; D.task_viol = D.task_viol > 0;
D.viol = D.m > 1;
D.wbin = discretize(D.X(:, strcmp(D.names, 'W')), [-inf 3 7 10 inf]);
end
