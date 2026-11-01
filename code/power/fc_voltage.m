function [V, Vact_ss] = fc_voltage(I, v_act, fc)
%FC_VOLTAGE Dien ap stack FC.
%   [V, Vact_ss] = FC_VOLTAGE(I, v_act, fc)
%   Duong phan cuc AeroStack [R24, Eq. 1]: V = V0 - B ln(I) - R I - m exp(n I),
%   quy ve mot cell roi nhan Ncell; dong co theo dien tich (I_ref = I/scale).
%   Thanh phan kich hoat B ln(I) di qua khau tre bac 1 tau = Td/3 [R19, Eq. 1]:
%   v_act la trang thai cua khau tre, Vact_ss la gia tri xac lap.
r = fc.ref;
Ir = max(I/fc.scale, 0.05);             % GT: chan duoi ln(I)
Vact_ss = r.B*log(Ir);
V = (r.V0 - v_act - r.R*Ir - r.m*exp(r.n*Ir))*fc.Ncell/r.Ncell;
end
