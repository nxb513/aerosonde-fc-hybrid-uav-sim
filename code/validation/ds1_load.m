function d = ds1_load(file)
%DS1_LOAD Doc mot maneuver cua bo du lieu bay X8 [DS1].
%   d = DS1_LOAD(file) voi file .mat cua [DS1] (bien 'maneuver', 41 cot).
%   [DS1] Low-Hansen & Hann, DataverseNO V1, 2024, doi:10.18710/U4TLYV (CC0).
%   Ten cot theo 00_README.txt cua bo du lieu. Ghi chu khi doc du lieu:
%     - GPS_lon/GPS_lat thuc te tinh bang rad (0.1697 rad = 9.72 deg E, Agdenes),
%       README ghi "degree".
%     - IndicatedSpeed = van toc khi da hieu chinh tu van toc quan tinh va gio
%       uoc luong (co gio dung) -> dung lam Va do.
%     - elevator/aileron la lenh (rad), throttle la lenh [0, 1].

names = {'t','elevator','aileron','throttle', ...
    'acc_x','acc_y','acc_z','imu_p','imu_q','imu_r', ...
    'phi','theta','psi','p','q','r','u','v','w','vn','ve','vd', ...
    'wn','we','wd','alpha','beta','TrueSpeed','Va', ...
    'lon','lat','height','gps_x','gps_y','gps_z','cog','sog', ...
    'Current','Voltage','Temperature','Pressure'};
S = load(file, 'maneuver');
M = S.maneuver;
if size(M, 2) ~= numel(names)
    error('ds1_load:cols', 'Mong doi %d cot, nhan %d.', numel(names), size(M, 2));
end
for k = 1:numel(names)
    d.(names{k}) = M(:, k);
end
[~, d.name] = fileparts(file);
end
