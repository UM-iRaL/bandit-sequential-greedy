clear;
close;
% Experiment parameters
num_rep = 1;
run_len = 200;
num_robot = 1;
num_tg = 1;
map_size = 70;

% Action set for robots
[V,W] = meshgrid([3,1],[0, -1, 1, -2, 2]);
ACTION_SET = transpose([V(:), W(:)]);

% Visibility map
vis_map = init_blank_ndmap([-map_size; -map_size],[map_size; map_size],0.25,'logical');
%vis_map.map = ~vis_map.map;
vis_map_save = cell(run_len,num_rep);

% Initial pose for robots
x_true = zeros(run_len+1, num_robot,3,num_rep); % robots
x_true(1, 1,:, :) = repmat([0;-20;0],1,num_rep);

% Initial position for targets
tg_true = zeros(3,num_tg,run_len+1,num_rep); % dynamic target
tg_true(:,1,1,:) = repmat([-36;-20;1],1,num_rep);

% Measurement History Data
z_d_save = cell(run_len,num_robot,num_rep); % target measurements
u_save = zeros(run_len,num_robot,2,num_rep); % control

% Esitimate Data
tg_save = cell(run_len,num_rep);
tg_cov_save = cell(run_len,num_rep);

% Should we get video and image?
vid = false;
viz = true;


for rep = 1:num_rep
    % Setting

    % Create Robots and Planners
    for r = 1:num_robot
        R(r) = robot_nx(x_true(1, r, :, :));
        %P(r) = bsg_planner();
    end
    % Visualization
    if viz
        figure('Color',[1 1 1],'Position',[100,277,1200,800]);
        hold on;
        h0.viz = imagesc([vis_map.pos{1}(1);vis_map.pos{1}(end)],...
            [vis_map.pos{2}(1);vis_map.pos{2}(end)],vis_map.map.');
        cbone = bone; colormap(cbone(end:-1:(end-30),:));
              
        axis([-map_size,map_size,-map_size,map_size]);
        for r = 1:num_robot
            h0.rob(r) = draw_pose_nx([],permute(x_true(1,r,:,rep),[3 2 1]),'g',2.2);
            h0.fov(r) = draw_fov_nx([],permute(x_true(1,r,:,rep),[3 2 1]),R(r).fov,R(r).r_sense);
        end
        %h0.xe = draw_traj_nx([],permute(x_save(1,:,:,rep),[1 3 2]),'r:');
        h0.tg_cov = [];
        title(sprintf('Time Step: %d',0));
        xlabel('x [m]','FontSize',14);
        ylabel('y [m]','FontSize',14);
        drawnow;
        if vid
            
        end
    end

    
    for t = 1:run_len

        % Plan Moves
        for r = 1:num_robot
            u_save(t, r, :, rep) = ACTION_SET(:,2);
        end
        % Move Robots
        for r = 1:num_robot
            R(r).move(squeeze(u_save(t, r, :, rep)));
        end
        % Move Targets
        for tg = 1:num_tg

        end
        % Log Mearsurement
            % 1. Maintain a map from robots to detected targets
            % 2. Compute covariance
        % Map
        tg_cov_save{t, rep} = rand(2,2, num_tg);
        tg_save{t, rep} = zeros(2, num_tg);
        for r = 1:num_robot
            x_true(t+1,r,:,rep) = R(r).get_x();
            %R(r).get_x()
        end
            
            
            
        % Receive Loss -> Update Loss
            % 1. Compute objective function
            % 2. Compute loss
            % 3. Pass loss to bsg_planner()
        
            
            
            
            
            
            
            
        % Visualization
        if viz
            set(h0.viz,'cdata',vis_map.map.');

            h0.y = draw_traj_nx([],permute(tg_true(:,:,1:t,rep),[3 1 2 4]),'b:');

            for r = 1:num_robot
                h0.rob(r) = draw_pose_nx(h0.rob(r),permute(x_true(t,r,:,rep),[3 2 1]),'g',2.2);
                h0.fov(r) = draw_fov_nx(h0.fov(r),permute(x_true(t,r,:,rep),[3 2 1]),R(r).fov,R(r).r_sense);
            end
            
            tmp = tg_save{t, rep};
            if ~isempty(tmp)
                h0.tg_cov = draw_covariances_nx(h0.tg_cov, tmp(:,1), tg_cov_save{t,rep},'m');
            end
            title(sprintf('Time Step: %d',t));
           
            drawnow;
            if vid
                
            end
        end
    end
end
% Plot Measurement
