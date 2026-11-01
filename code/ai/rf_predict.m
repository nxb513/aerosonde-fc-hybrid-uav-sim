function y = rf_predict(P, X)
%RF_PREDICT Du bao trung binh cua rung dong goi (= predict cua TreeBagger hoi quy).
node = rf_leaf(P, X);
y = mean(reshape(P.val(node), size(node)), 2);
end
