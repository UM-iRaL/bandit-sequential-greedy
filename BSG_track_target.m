clear;
close;
% Experiment parameters
num_rep = 1;
run_len = 200;
num_robot = 1;
num_tg = 1;

% Action set for robots
[V,W] = meshgrid([3,1],[0, -1, 1, -2, 2]);
ACTION_SET = transpose([V(:), W(:)]);

% Initial pose for robots
x_true = zeros(run_len+1, num_robot,3,num_rep); % robots
x_true(1, 1,:, :) = repmat([0;-20;0],1,num_rep);

% Initial position for targets
tg_true = zeros(3,num_tg,run_len+1,num_rep); % dynamic target
tg_true(:,1,1,:) = repmat([-36;-20;1],1,num_rep);

% Measurement History Data
z_d_save = cell(run_len,num_robot,num_rep); % target measurement
u_save = zeros(run_len,num_robot,2,num_rep); % control

% Should we get video and image?
vid = false;
vis = false;


for rep = 1:num_rep
    % Setting

    % Create Robots and Planners
    for r = 1:num_robot
        R(r) = robot_nx(x_true(1, r, :, :));
        %P(r) = bsg_planner();
    end
    if viz
        
        if vid

        end
    end

    
    for t = 1:num_iter

        % Plan Moves
        for r = 1:num_robot
            R(r).move(u_save(t, r, :, rep));
        end
        % Move Targets
        for tg = 1:num_tg

        end
        % Log Mearsurement
            % 1. Maintain a map from robots to detected targets
            % 2. Compute covariance

        % Receive Loss -> Update Loss
            % 1. Compute objective function
            % 2. Compute loss
            % 3. Pass loss to bsg_planner()

    end
end
% Plot Measurement
