function M = afp_fit(X, y, tau, opts)
%AFP_FIT Hoi quy luong tu bang rung ngau nhien (quantile regression forest).
%   M = AFP_FIT(X, y, tau, opts): luong tu tren tau cua bien m.
%   Dung TreeBagger + quantilePredict (Statistics and Machine Learning Toolbox).
%   Ly do chon: hoi quy luong tu phu hop CQR hon MSE + sigma [R32]; rung cay
%   khong can chuan hoa dac trung va on dinh voi dac trung roi rac.
if nargin < 3, tau = 0.9; end
if nargin < 4, opts = struct(); end
if ~isfield(opts, 'ntrees'), opts.ntrees = 200; end
if ~isfield(opts, 'leaf'), opts.leaf = 5; end
if ~isfield(opts, 'cols'), opts.cols = 1:size(X, 2); end
rng(42);
M.B = TreeBagger(opts.ntrees, X(:, opts.cols), y, 'Method', 'regression', ...
    'MinLeafSize', opts.leaf, 'NumPredictorsToSample', max(1, round(numel(opts.cols)/3)));
M.tau = tau; M.cols = opts.cols;
end
