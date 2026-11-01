function [cmd, st, info] = autopilot_step(ap, st, meas, ref, phi_max, dt)
%AUTOPILOT_STEP Mot buoc autopilot: roll/course (aileron), pitch/altitude (elevator),
%   airspeed (ga), yaw damper washout (rudder) [R37, ch. 6].
%   meas: phi theta p q r chi h Va;  ref: chi_c h_c Va_c;  phi_max: bank toi da.
%   Anti-windup: dung tich phan khi dau ra vong bi bao hoa (GT).
%   cmd = [de; da; dr; dt].
if isempty(st), st = struct('i_chi', 0, 'i_h', 0, 'i_V', 0, 'x_wo', 0); end

e_chi = mod(ref.chi_c - meas.chi + pi, 2*pi) - pi;
phi_u = ap.kp_chi*e_chi + ap.ki_chi*st.i_chi;
phi_c = min(max(phi_u, -phi_max), phi_max);
if phi_c == phi_u, st.i_chi = st.i_chi + e_chi*dt; end

e_h = ref.h_c - meas.h;
th_u = ap.trim.theta + ap.kp_h*e_h + ap.ki_h*st.i_h;
th_c = min(max(th_u, ap.theta_lim(1)), ap.theta_lim(2));
if th_c == th_u, st.i_h = st.i_h + e_h*dt; end

e_V = ref.Va_c - meas.Va;
dt_u = ap.trim.dt + ap.kp_V*e_V + ap.ki_V*st.i_V;
dtc = min(max(dt_u, 0), 1);
if dtc == dt_u, st.i_V = st.i_V + e_V*dt; end

r_hp = meas.r - ap.p_wo*st.x_wo;          % washout: s/(s + p_wo)
st.x_wo = st.x_wo + dt*(meas.r - ap.p_wo*st.x_wo);

de = ap.trim.de + ap.kp_th*(th_c - meas.theta) - ap.kd_th*meas.q;
da = ap.trim.da + ap.kp_phi*(phi_c - meas.phi) - ap.kd_phi*meas.p;
dr = ap.trim.dr + ap.k_r*r_hp;
cmd = [de; da; dr; dtc];
info.phi_c = phi_c; info.th_c = th_c;
info.sat_phi = phi_c ~= phi_u; info.sat_th = th_c ~= th_u; info.sat_dt = dtc ~= dt_u;
end
