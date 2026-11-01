function I = x8_state_index()
%X8_STATE_INDEX Chi so cac thanh phan trong vector trang thai 19 chieu.
%   pos (NED, m) | v_b (m/s) | q (than->NED) | om_b (rad/s) | Om (rad/s)
%   | elevon [der; del] (rad) | toc do elevon (rad/s) | ga thuc te
I.pos  = 1:3;
I.vel  = 4:6;
I.q    = 7:10;
I.om   = 11:13;
I.Om   = 14;
I.dlt  = 15:16;
I.ddlt = 17:18;
I.thr  = 19;
I.n    = 19;
end
