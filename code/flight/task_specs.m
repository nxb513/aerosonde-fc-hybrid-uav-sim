function specs = task_specs(seed, W_lvl, n)
%TASK_SPECS Chuoi nhiem vu va gio theo tung nhiem vu cho mot chuyen bay E3.
%   specs = TASK_SPECS(seed, W_lvl, n): n dac ta (mac dinh 400, du cho chuyen bay dai nhat).
%   Moi nhiem vu rut cung so luong so ngau nhien bat ke loai, nen cung seed cho cung
%   chuoi nhiem vu + gio voi moi chinh sach (so sanh cap doi).
%   Loai: 1 chuyen chang, 2 doi huong, 3 leo/ha, 4 bay vong cho, 5 quet luong cay,
%         6 giu quy dao trong gio giat (task_make).
%   Gio (GT, "mua" = muc gio W_lvl): toc do trung binh U[W_lvl - 2, W_lvl + 2] (>= 0),
%   huong troi U[-30, 30] deg giua hai nhiem vu; he so cuong do nhieu Dryden U[0.5, 3]
%   (cong thuc cuong do theo [R12], he so nhan la GT).
if nargin < 3, n = 400; end
rs = RandStream('mt19937ar', 'Seed', 7e4 + seed);
d = 360*rand(rs);
specs = struct('type', cell(n, 1), 'u', [], 'W', [], 'dir', [], 'turb', []);
for k = 1:n
    specs(k).type = randi(rs, 6);
    specs(k).u = rand(rs, 1, 4);
    uw = rand(rs, 1, 3);
    d = mod(d + 60*(uw(2) - 0.5), 360);
    specs(k).W = max(W_lvl + 4*(uw(1) - 0.5), 0);
    specs(k).dir = d;
    specs(k).turb = 0.5 + 2.5*uw(3);
end
end
