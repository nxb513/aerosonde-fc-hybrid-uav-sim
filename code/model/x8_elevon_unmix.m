function [der, del] = x8_elevon_unmix(de, da)
%X8_ELEVON_UNMIX Nghich dao cua x8_elevon_mix.
der = de - da;
del = de + da;
end
