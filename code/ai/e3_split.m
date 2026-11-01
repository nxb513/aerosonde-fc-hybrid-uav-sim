function sp = e3_split(flight)
%E3_SPLIT Chia mau theo chuyen bay: 60 % huan luyen, 20 % hieu chuan, 20 % kiem tra.
%   sp = E3_SPLIT(flight); tr/ca/te la mat na logic tren cac dong; f_tr/f_ca/f_te la id
%   chuyen bay. Hoan vi co dinh (seed 11) de moi lan huan luyen dung cung mot cach chia.
f = unique(flight);
rs = RandStream('mt19937ar', 'Seed', 11);
f = f(randperm(rs, numel(f)));
n = numel(f); a = round(0.6*n); b = round(0.8*n);
sp.f_tr = sort(f(1:a)); sp.f_ca = sort(f(a+1:b)); sp.f_te = sort(f(b+1:end));
sp.tr = ismember(flight, sp.f_tr); sp.ca = ismember(flight, sp.f_ca); sp.te = ismember(flight, sp.f_te);
end
