function [E, Rp] = bat_voltage(it, istar, bat)
%BAT_VOLTAGE Suat dien dong va dien tro pack theo mo hinh Tremblay [R4].
%   V_pack = E - Rp*I (I > 0 khi xa). it: Ah da xa moi cell; istar: dong loc moi cell (A).
%   Xa:  V = E0 - R i - K Q/(Q-it) (it + i*) + A exp(-B it)
%   Nap: V = E0 - R i - K Q/(it+0.1Q) i* - K Q/(Q-it) it + A exp(-B it)
%   [R4] ghi mau so (it - 0.1Q); mau so nay doi dau khi it < 0.1Q (pin gan day)
%   lam dien ap nap giam. Ta dung (it + 0.1Q) de giu dung chieu vat ly (GT).
c = bat.cell;
itc = min(max(it, 0), 0.999*c.Q);
pol = c.K*c.Q/(c.Q - itc);
if istar >= 0
    Ec = c.E0 - pol*(itc + istar) + c.A*exp(-c.B*itc);
else
    Ec = c.E0 - c.K*c.Q/(itc + 0.1*c.Q)*istar - pol*itc + c.A*exp(-c.B*itc);
end
E  = bat.Ns*Ec;
Rp = bat.Ns*c.R/bat.Np;
end
