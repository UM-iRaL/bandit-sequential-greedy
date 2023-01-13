clear;
close;
num_rep = 1;
run_len = 200;
num_robot = 1;

% action set for robots
[V,W] = meshgrid([3,1],[0, -1, 1, -2, 2]);
ACTION_SET = transpose([V(:), W(:)]);

z_d_save = cell(run_len,num_robot,num_rep); % target measurement

% should we get video and image?
vid = false;
vis = false;

for rep = 1:num_rep
    % Setting

    for t = 1:num_iter
        % Create Robots and Planners
        for r = 1:num_robot
            %R(r) = robot_nx();
            %P(r) = bsg_planner();
        end
        % Plan Moves

        % Move Targets

        % Log Mearsurement
        
        % Receive Loss -> Update Loss
       
    end
end
% Plot Measurement
