function [de, da] = x8_elevon_mix(der, del)
%X8_ELEVON_MIX Elevon phai/trai -> elevator/aileron tuong duong [R2, Eq. 14].
%   [de; da] = 1/2*[1 1; -1 1]*[der; del]   (cung quy uoc [R37, muc 4.2.1])
de = 0.5*(der + del);
da = 0.5*(-der + del);
end
