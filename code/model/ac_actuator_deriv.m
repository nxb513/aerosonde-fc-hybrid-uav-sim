function [d_dot, dd_dot, th_dot] = ac_actuator_deriv(d, dd, th, cmd, p)
%AC_ACTUATOR_DERIV Servo bac 2 co bao hoa goc/toc do cho [de; da; dr]; ga bac 1.
%   dd_dot = wn^2 (sat(d_c) - d) - 2 zeta wn dd  (wn, zeta, tre [R2, Bang 9]; GT cho Aerosonde)
%   Bao hoa: lenh kep trong +-max; khi cham gioi han goc/toc do va con day ra ngoai
%   thi dao ham bang 0 (GT). Ga: (sat(dt_c) - dt)/tau.
a = p.act;
dc = min(max(cmd(1:3), -a.max), a.max);
d_dot = min(max(dd, -a.rate), a.rate);
dd_dot = a.wn^2*(dc - d) - 2*a.zeta*a.wn*dd;
for k = 1:3
    if (dd(k) >= a.rate(k) && dd_dot(k) > 0) || (dd(k) <= -a.rate(k) && dd_dot(k) < 0)
        dd_dot(k) = 0;
    end
    if (d(k) >= a.max(k) && d_dot(k) > 0) || (d(k) <= -a.max(k) && d_dot(k) < 0)
        d_dot(k) = 0;
    end
end
th_dot = (min(max(cmd(4), 0), 1) - th)/a.thr_tau;
end
