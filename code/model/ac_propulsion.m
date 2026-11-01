function out = ac_propulsion(Va, delta_t, Ub, p)
%AC_PROPULSION Dong co DC + canh quat o trang thai xac lap, noi bus DC.
%   out = AC_PROPULSION(Va, delta_t, Ub, p)
%   [R37, Eq. 4.17-4.21]: toc do canh quat Omega la nghiem duong cua can bang momen
%     a Om^2 + b Om + c = 0, a = rho D^5 CQ0/(2pi)^2,
%     b = rho D^4 CQ1 Va/(2pi) + KQ KV/R, c = rho D^3 CQ2 Va^2 - KQ Vin/R + KQ i0
%   Dien ap vao dong co: Vin = delta_t * Ub. [R37, Eq. 4.22] dung Vmax*delta_t; ta thay
%   Vmax bang dien ap bus thuc te tu he nguon (mo rong cua ta, cung y [R2, Eq. 28]).
%   Dong dong co I = (Vin - KV Om)/R [R37, Eq. 4.19]; cong suat P = Vin*I;
%   dong bus Ibus = P/Ub (GT: ESC khong ton hao, khong hoi sinh).
%   GT: Om < Om_min -> canh quat tu do, T = Q = 0; he so tien kep J <= J_max.
pr = p.prop; rho = p.env.rho; D = pr.D;
Vin = max(delta_t, 0)*Ub;
a = rho*D^5*pr.CQ(1)/(2*pi)^2;
b = rho*D^4*pr.CQ(2)*Va/(2*pi) + pr.KQ*pr.KV/pr.R;
c = rho*D^3*pr.CQ(3)*Va^2 - pr.KQ*Vin/pr.R + pr.KQ*pr.i0;
disc = b^2 - 4*a*c;
if disc > 0
    Om = (-b + sqrt(disc))/(2*a);
else
    Om = 0;
end
if Om < pr.Om_min
    out = struct('Om', 0, 'J', NaN, 'T', 0, 'Q', 0, 'I', 0, 'Pel', 0, 'Ibus', 0, 'Vin', Vin);
    return
end
J = min(2*pi*Va/(Om*D), pr.J_max);
CT = pr.CT(1) + pr.CT(2)*J + pr.CT(3)*J^2;
CQ = pr.CQ(1) + pr.CQ(2)*J + pr.CQ(3)*J^2;
T = rho*D^4/(4*pi^2)*CT*Om^2;
Q = rho*D^5/(4*pi^2)*CQ*Om^2;
I = max((Vin - pr.KV*Om)/pr.R, 0);
Pel = Vin*I;
out = struct('Om', Om, 'J', J, 'T', T, 'Q', Q, 'I', I, 'Pel', Pel, ...
             'Ibus', Pel/max(Ub, eps), 'Vin', Vin);
end
