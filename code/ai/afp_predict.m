function q = afp_predict(M, X)
%AFP_PREDICT Luong tu tren du bao cua bien m (chua hieu chuan).
q = quantilePredict(M.B, X(:, M.cols), 'Quantile', M.tau);
end
