function S = surrogate_fit(T1)
%SURROGATE_FIT Mo hinh thay the (rung ngau nhien) du bao ket qua cua mot ung vien.
%   S = SURROGATE_FIT(T1) huan luyen tren E1: xac suat hoan thanh, thoi gian
%   hoan thanh, nang luong dien, cong suat dinh, suy giam pin, bien thien cong
%   suat FC. Dung cung dac trung truoc chuyen bay voi AFP (afp_dataset).
D = afp_dataset(T1);
ok = D.ok;
rng(7);
opt = {'Method', 'regression', 'MinLeafSize', 5};
S.complete = TreeBagger(150, D.X(ok, :), double(T1.completed(ok)), opt{:});
c = ok & T1.completed;
S.t_done = TreeBagger(150, D.X(c, :), T1.t_done(c), opt{:});
S.E_el   = TreeBagger(150, D.X(ok, :), T1.E_el(ok), opt{:});
S.P_peak = TreeBagger(150, D.X(ok, :), T1.P_peak(ok), opt{:});
S.bat_dmg = TreeBagger(150, D.X(ok, :), T1.bat_dmg(ok), opt{:});
S.fc_dP  = TreeBagger(150, D.X(ok, :), T1.fc_dP(ok), opt{:});
end
