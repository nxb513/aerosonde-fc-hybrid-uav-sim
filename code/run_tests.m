function results = run_tests()
%RUN_TESTS Chay toan bo test; loi neu co test hong (dung cho CI).
setup_paths();
root = fileparts(mfilename('fullpath'));
results = runtests(fullfile(root, 'tests'));
disp(table(results));
assertSuccess(results);
end
