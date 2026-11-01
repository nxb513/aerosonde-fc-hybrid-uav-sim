function out = ac_aero(vr_b, om_b, dl, p)
%AC_AERO Luc/momen khi dong Aerosonde tai CG danh dinh (he than).
%   out = AC_AERO(vr_b, om_b, dl, p), dl = [de; da; dr] (rad).
%   Mo hinh [R37, Eq. 4.6-4.16, 4.24-4.26]:
%     C_L(a) tron tuyen tinh + tam phang qua sigma(a; M, alpha0) [R37, Eq. 4.9-4.10]
%     C_D(a) = C_Dp + (C_L0 + C_La a)^2/(pi e AR)                [R37, Eq. 4.11]
%     F_lift = qS[C_L(a) + C_Lq c q/(2Va) + C_Lde de]           [R37, Eq. 4.6]
%     F_drag = qS[C_D(a) + C_Dq c q/(2Va) + C_Dde |de|]         [R37, Eq. 4.7]
%     luc ngang, momen lan/chuc/lech theo [R37, Eq. 4.14-4.16, 4.26]
A = p.aero; g = p.geom;
ua = vr_b(1); va = vr_b(2); wa = vr_b(3);
Va = max(sqrt(ua^2 + va^2 + wa^2), A.Va_min);
al = atan2(wa, ua);
be = asin(max(min(va/Va, 1), -1));
de = dl(1); da = dl(2); dr = dl(3);
qs = om_b(2)*g.c/(2*Va); ps = om_b(1)*g.b/(2*Va); rs = om_b(3)*g.b/(2*Va);

e1 = exp(-A.M*(al - A.alpha0)); e2 = exp(A.M*(al + A.alpha0));
sig = (1 + e1 + e2)/((1 + e1)*(1 + e2));
CLa = (1 - sig)*(A.CL0 + A.CLa*al) + sig*2*sign(al)*sin(al)^2*cos(al);
CDa = A.CDp + (A.CL0 + A.CLa*al)^2/(pi*g.e*g.AR);
CL = CLa + A.CLq*qs + A.CLde*de;
CD = CDa + A.CDq*qs + A.CDde*abs(de);
CY = A.CY0 + A.CYb*be + A.CYp*ps + A.CYr*rs + A.CYda*da + A.CYdr*dr;
Cl = A.Cl0 + A.Clb*be + A.Clp*ps + A.Clr*rs + A.Clda*da + A.Cldr*dr;
Cm = A.Cm0 + A.Cma*al + A.Cmq*qs + A.Cmde*de;
Cn = A.Cn0 + A.Cnb*be + A.Cnp*ps + A.Cnr*rs + A.Cnda*da + A.Cndr*dr;

ca = cos(al); sa = sin(al);
CX = -CD*ca + CL*sa; CZ = -CD*sa - CL*ca;
qbar = 0.5*p.env.rho*Va^2;
out.F = qbar*g.S*[CX; CY; CZ];
out.M = qbar*g.S*[g.b*Cl; g.c*Cm; g.b*Cn];
out.Va = Va; out.alpha = al; out.beta = be; out.qbar = qbar;
out.CL = CL; out.CD = CD;
end
