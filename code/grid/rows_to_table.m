function T = rows_to_table(rows)
%ROWS_TO_TABLE Gop cac dong ket qua (struct) thanh bang, dong nhat ten truong
%   (kich ban loi co it truong hon: so -> NaN, msg -> "").
rows = rows(~cellfun(@isempty, rows));
if isempty(rows), T = table(); return; end
allf = {};
for k = 1:numel(rows), allf = union(allf, fieldnames(rows{k}), 'stable'); end
for k = 1:numel(rows)
    for j = 1:numel(allf)
        if ~isfield(rows{k}, allf{j})
            if strcmp(allf{j}, 'msg'), rows{k}.(allf{j}) = ""; else, rows{k}.(allf{j}) = NaN; end
        end
    end
    rows{k} = orderfields(rows{k}, allf);
end
T = struct2table([rows{:}]');
end
