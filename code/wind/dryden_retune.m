function w = dryden_retune(w, W_h, h, intensity)
%DRYDEN_RETUNE Doi cuong do nhieu Dryden giua chuyen bay, giu nguyen trang thai bo loc.
%   w = DRYDEN_RETUNE(w, W_h, h, intensity). Do dai thang L chi phu thuoc h nen cac bo
%   loc roi rac giu nguyen; chi doi ma tran ra C = sigma*C1 (dryden_sigma [R12]).
w.sigma = dryden_sigma(W_h, h, intensity);
for k = 1:3
    w.sys{k}.C = w.sigma(k)*w.sys{k}.C1;
end
end
