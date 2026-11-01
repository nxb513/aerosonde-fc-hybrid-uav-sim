function w = dryden_init(W_h, dir_deg, h, intensity, Va, dt, seed)
%DRYDEN_INIT Gio trung binh + nhieu Dryden tang thap (MIL-F-8785C [R12]).
%   w = DRYDEN_INIT(W_h, dir_deg, h, intensity, Va, dt, seed)
%     W_h       toc do gio trung binh tai do cao bay (m/s)
%     dir_deg   huong gio thoi TOI (khi tuong, 0 = gio thoi tu Bac xuong Nam)
%     h         do cao bay (m); intensity he so nhan cuong do nhieu (GT, luoi)
%   Cuong do va do dai thang: dryden_sigma [R12]. Ham truyen Dryden theo [R37, muc 4.4],
%   tinh o Va danh dinh; nhieu trang dau vao co mat do pho sao cho phuong sai dau ra
%   = sigma^2. C1 = ma tran ra ung voi sigma = 1 (de dryden_retune doi cuong do).
[w.sigma, w.L] = dryden_sigma(W_h, h, intensity);
w.mean_ned = -W_h*[cosd(dir_deg); sind(dir_deg); 0];   % gio thoi tu huong dir
w.Va = Va; w.dt = dt;
w.rng = RandStream('mt19937ar', 'Seed', seed);
% Roi rac hoa chinh xac (ZOH) cac bo loc
w.sys = cell(1, 3);
for k = 1:3
    V = Va; L = w.L(k); s = w.sigma(k);
    if k == 1
        A = -V/L; B = 1; C = s*sqrt(2*V/(pi*L)); C1 = sqrt(2*V/(pi*L)); D = 0;
    else
        a = V/L; b = V/(sqrt(3)*L);
        A = [0 1; -a^2 -2*a]; B = [0; 1];
        C = s*sqrt(3*V/(pi*L))*[b 1]; C1 = sqrt(3*V/(pi*L))*[b 1]; D = 0;
    end
    n = size(A, 1);
    M = expm([A B; zeros(1, n + 1)]*dt);
    w.sys{k} = struct('Ad', M(1:n, 1:n), 'Bd', M(1:n, n+1), 'C', C, 'C1', C1, 'D', D, 'x', zeros(n, 1));
end
w.gust_b = zeros(3, 1);
end
