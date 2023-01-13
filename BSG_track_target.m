clear;
close;
num_rep = 1;
run_len = 200;
num_robot = 1;

% action set for robots
[V,W] = meshgrid([3,1],[0, -1, 1, -2, 2]);
ACTION_SET = transpose([V(:), W(:)]);

z_d_save = cell(run_len,num_robot,num_rep); % target measurement
