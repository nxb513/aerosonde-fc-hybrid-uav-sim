function r = bat_damage_rate(crate, bat)
%BAT_DAMAGE_RATE Ty le mat dung luong (%) tren moi Ah thong qua (chuan hoa 2 Ah/cell).
%   Mo hinh [R18, Eq. 7]: Q_loss = B(c) exp[(-31700 + 370.3 c)/(R T)] Ah^0.55.
%   Dang bien (marginal) tai muc thong qua tham chieu Ah_ref (GT):
%     dQ/dAh = 0.55 B(c) exp(.) Ah_ref^(-0.45)
%   B(c) noi suy tuyen tinh theo c giua cac diem cua [R18, Bang 3].
%   Ah cua [R18] tinh cho cell 2 Ah -> nhan 2/Q_cell de quy doi.
a = bat.age;
c = min(max(crate, a.c(1)), a.c(end));
B = interp1(a.c, a.B, c);
Rg = 8.314;
r = a.z*B*exp((-a.Ea0 + a.Ea1*c)/(Rg*a.T))*a.Ah_ref^(a.z - 1)*(2/bat.cell.Q);
end
