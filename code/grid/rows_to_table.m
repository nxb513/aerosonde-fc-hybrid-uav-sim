function T = rows_to_table(rows)
%ROWS_TO_TABLE Gop cac dong ket qua (struct) thanh bang, dong nhat ten truong
%   (chuyen bay loi co it truong hon: so -> NaN, chuoi -> "").
rows = rows(~cellfun(@isempty, rows));
if isempty(rows), T = table(); return; end
allf = {}; isstr = {};
for k = 1:numel(rows)
    f = fieldnames(rows{k});
    allf = union(allf, f, 'stable');
    isstr = union(isstr, f(cellfun(@(n) isstring(rows{k}.(n)), f)));
end
for k = 1:numel(rows)
    for j = 1:numel(allf)
        if ~isfield(rows{k}, allf{j})
            if ismember(allf{j}, isstr), rows{k}.(allf{j}) = ""; else, rows{k}.(allf{j}) = NaN; end
        end
    end
    rows{k} = orderfields(rows{k}, allf);
end
T = struct2table([rows{:}]');
end
