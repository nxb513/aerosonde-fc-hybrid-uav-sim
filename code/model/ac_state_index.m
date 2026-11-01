function I = ac_state_index()
%AC_STATE_INDEX Chi so vector trang thai 20 chieu.
%   pos (NED) | v_b (van toc dat, he than) | q (than->NED) | om_b
%   | goc lai [de; da; dr] | toc do lai | ga thuc te
I.pos  = 1:3;
I.vel  = 4:6;
I.q    = 7:10;
I.om   = 11:13;
I.dlt  = 14:16;
I.ddlt = 17:19;
I.thr  = 20;
I.n    = 20;
end
