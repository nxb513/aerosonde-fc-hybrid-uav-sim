function varargout = quat_utils(fn, varargin)
%QUAT_UTILS Ham quaternion dung chung (vo huong dung dau, q = [q0; q1; q2; q3]).
%   R  = quat_utils('dcm', q)          ma tran quay than -> NED (R_nb)
%   q  = quat_utils('from_euler', eul) eul = [phi; theta; psi], thu tu 3-2-1
%   eul= quat_utils('to_euler', q)
%   dq = quat_utils('deriv', q, om)    dq/dt = 1/2 * q (x) [0; om]
%   Quy uoc Euler 3-2-1 nhu [R37, Eq. 2.4]; khong phu thuoc toolbox.
switch fn
    case 'dcm'
        varargout{1} = q2dcm(varargin{1});
    case 'from_euler'
        varargout{1} = eul2q(varargin{1});
    case 'to_euler'
        varargout{1} = q2eul(varargin{1});
    case 'deriv'
        q = varargin{1}; w = varargin{2};
        Om = [0 -w(1) -w(2) -w(3);
              w(1) 0 w(3) -w(2);
              w(2) -w(3) 0 w(1);
              w(3) w(2) -w(1) 0];
        varargout{1} = 0.5*Om*q;
    otherwise
        error('quat_utils:fn', 'Ham khong ho tro: %s', fn);
end
end

function R = q2dcm(q)
q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
R = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3),     2*(q1*q3+q0*q2);
     2*(q1*q2+q0*q3),     q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
     2*(q1*q3-q0*q2),     2*(q2*q3+q0*q1),     q0^2-q1^2-q2^2+q3^2];
end

function q = eul2q(e)
cph = cos(e(1)/2); sph = sin(e(1)/2);
cth = cos(e(2)/2); sth = sin(e(2)/2);
cps = cos(e(3)/2); sps = sin(e(3)/2);
q = [cph*cth*cps + sph*sth*sps;
     sph*cth*cps - cph*sth*sps;
     cph*sth*cps + sph*cth*sps;
     cph*cth*sps - sph*sth*cps];
end

function e = q2eul(q)
q = q/norm(q);
q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
phi   = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2));
theta = asin(max(min(2*(q0*q2 - q1*q3), 1), -1));
psi   = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2));
e = [phi; theta; psi];
end
