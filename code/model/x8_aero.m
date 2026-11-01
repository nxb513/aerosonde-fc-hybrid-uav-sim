function out = x8_aero(vr_b, om_b, de, da, CT, p)
%X8_AERO Luc va momen khi dong X8 tai diem tham chieu khi dong (he than).
%   out = X8_AERO(vr_b, om_b, de, da, CT, p)
%     vr_b  van toc tuong doi gio trong he than [ua; va; wa] (m/s)
%     om_b  van toc goc [p; q; r] (rad/s)
%     de,da goc elevator/aileron tuong duong (rad), tu tron elevon [R2, Eq. 14]
%     CT    he so luc day canh quat (hoi quy trong C_D) [R2, Eq. 17b]
%
%   Mo hinh [R2, Eq. 15-21]: he so luc trong stability frame, momen trong he than.
%   Doi luc sang he than: CX = -CD cos(a) + CL sin(a), CZ = -CD sin(a) - CL cos(a)
%   [R2, Eq. 19-20][R37, Eq. 4.25].

A = p.aero; g = p.geom;
ua = vr_b(1); va = vr_b(2); wa = vr_b(3);
Va    = max(sqrt(ua^2 + va^2 + wa^2), A.Va_min);
alpha = atan2(wa, ua);
beta  = asin(max(min(va/Va, 1), -1));

ps = om_b(1)*g.b/(2*Va);
qs = om_b(2)*g.c/(2*Va);
rs = om_b(3)*g.b/(2*Va);

CL = A.CL0 + A.CLa*alpha + A.CLq*qs + A.CLde*de;
CD = A.CD0 + A.CDq*qs + A.CDCT*CT + A.CDk1*CL + A.CDk2*CL^2;
Cm = A.Cm0 + A.Cma*alpha + A.Cmq*qs + A.Cmde*de;
CY = A.CY0 + A.CYb*beta + A.CYp*ps + A.CYr*rs + A.CYda*da;
Cl = A.Cl0 + A.Clb*beta + A.Clp*ps + A.Clr*rs + A.Clda*da;
Cn = A.Cn0 + A.Cnb*beta + A.Cnp*ps + A.Cnr*rs + A.Cnda*da;

ca = cos(alpha); sa = sin(alpha);
CX = -CD*ca + CL*sa;
CZ = -CD*sa - CL*ca;

qbar = 0.5*p.env.rho*Va^2;
out.F = qbar*g.S*[CX; CY; CZ];
out.M = qbar*g.S*[g.b*Cl; g.c*Cm; g.b*Cn];
out.Va = Va; out.alpha = alpha; out.beta = beta; out.qbar = qbar;
out.C = struct('CL', CL, 'CD', CD, 'CY', CY, 'Cl', Cl, 'Cm', Cm, 'Cn', Cn);
end
