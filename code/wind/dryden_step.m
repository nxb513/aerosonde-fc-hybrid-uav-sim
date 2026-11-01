function w = dryden_step(w)
%DRYDEN_STEP Cap nhat gust Dryden trong he than mot buoc dt.
%   Dau vao nhieu trang co mat do pho hai phia pi (don vi rad/s) -> phuong sai roi
%   rac pi/dt, de phuong sai dau ra bang sigma^2 (tich phan |H|^2 cua [R37, muc 4.4]).
g = zeros(3, 1);
sd = sqrt(pi/w.dt);
for k = 1:3
    S = w.sys{k};
    n = sd*randn(w.rng);
    g(k) = S.C*S.x;
    S.x = S.Ad*S.x + S.Bd*n;
    w.sys{k} = S;
end
w.gust_b = g;
end
