function out = x8_propulsion(Va, Om, delta_t, Ub, p)
%X8_PROPULSION Canh quat + dong co dien X8 noi bus DC.
%   out = X8_PROPULSION(Va, Om, delta_t, Ub, p)
%     Va      van toc khi (m/s)
%     Om      toc do canh quat Omega_p (rad/s)
%     delta_t ga thuc te (0..1, sau servo)
%     Ub      dien ap bus DC (V) -- diem ghep voi he nguon FC/pin/sieu tu
%
%   Canh quat [R2, Eq. 5-9]:
%     J = 2*pi*Va/(Om*D), T = rho*D^4/(4 pi^2)*CT(J)*Om^2, Q = rho*D^5/(4 pi^2)*CQ(J)*Om^2
%   Dong co [R2, Eq. 28-29]:
%     U = delta_t*Ub, Im = (U - Om*KE)/R, Qm = (Im - Im0)*KE, dOm = (Qm - Q)/Ip
%   Cong suat: Pel = U*Im; dong bus Ibus = Pel/Ub (GT: ESC khong ton hao).

pr  = p.prop;
rho = p.env.rho;

J  = 2*pi*Va/(max(Om, pr.Om_min)*pr.D);
J  = min(max(J, 0), pr.J_max);
CT = pr.CT(1) + pr.CT(2)*J + pr.CT(3)*J^2 + pr.CT(4)*J^3;
CQ = pr.CQ(1) + pr.CQ(2)*J + pr.CQ(3)*J^2;

kT = rho*pr.D^4/(4*pi^2);
kQ = rho*pr.D^5/(4*pi^2);
T  = kT*CT*Om^2;
Q  = kQ*CQ*Om^2;

U  = delta_t*Ub;
Im = (U - Om*pr.KE)/pr.R;
if ~pr.allow_regen
    Im = max(Im, 0);
end
Qm = (Im - pr.Im0)*pr.KE;
dOm = (Qm - Q)/pr.Ip;
if Om <= 0 && dOm < 0
    dOm = 0;                    % canh quat khong quay nguoc
end

Pel = U*Im;
out.J = J; out.CT = CT; out.CQ = CQ;
out.T = T; out.Q = Q;
out.U = U; out.Im = Im; out.Qm = Qm; out.dOm = dOm;
out.Pel = Pel;
out.Ibus = Pel/max(Ub, eps);
end
