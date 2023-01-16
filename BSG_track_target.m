clear;
close;
% Experiment parameters
num_rep = 1;
run_len = 200;
num_robot = 2;
num_tg = 2;
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
x_true(1, 1, :, :) = repmat([0;-20;0],1,num_rep);
x_true(1, 2, :, :) = repmat([0; 0; pi/4],1,num_rep);

% Initial position for targets
tg_true = zeros(3,num_tg,run_len+1,num_rep); % dynamic target
% first two are position, last one is id
tg_true(:,1,1,:) = repmat([20;0;1],1,num_rep);
tg_true(:,2,1,:) = repmat([40;0;2],1,num_rep);

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
        % Sense
        for r = 1:num_robot
            % targets
            z_d_save{t, r, rep} = R(r).sense(tg_true(:, :, t, rep)');
        end
        
        % Plan Moves
        for r = 1:num_robot
            u_save(t, r, :, rep) = ACTION_SET(:,2);
            
        end
        % Move Robots
        for r = 1:num_robot
            R(r).move(squeeze(u_save(t, r, :, rep)));
            x_true(t+1,r,:,rep) = R(r).get_x();
        end
        % Move Targets
        for kk = 1:num_tg
            tg_true(:,kk, t+1, rep) = tg_true(:, kk, t, rep);
        end
        % Log Mearsurement
            % 1. Maintain a map from robots to detected targets
            % 2. Compute covariance based on map
            
            
        % Key is robot id, Value is a collection of target ids
        target_map = containers.Map('KeyType','double','ValueType','any'); 
        for r = 1:num_robot
            Z_d = z_d_save{t, r, rep};
%             if size(Z_d, 1) ~= 0
%                 tg_ids = Z_d(:, end);
%                 target_map(r) = tg_ids;
%             end
            target_map(r) = Z_d;
        end
        estm_tg = zeros(2, num_tg);
        estm_tg_cov = zeros(2, 2, num_tg);
        % Compute variance
        for r = 1:num_robot
            msrmnt_rb = target_map(r);
            if size(msrmnt_rb, 1) == 0
                continue;
            end
            for k = 1 : size(msrmnt_rb, 1)
                % third column labels classes of targets.
                target_id = msrmnt_rb(k, end);

                if(estm_tg_cov(:,:,target_id) == zeros(2,2)) 
                    % first measurement
                    estm_tg(:, target_id) = inverse_rb(squeeze(x_true(t, r, :, rep))', msrmnt_rb(k,1:2))';
                    cov_z = [R(r).r_sigma 0; 0 R(r).b_sigma];
                    estm_tg_cov(:, :, target_id) = inv_rb_cov(squeeze(x_true(t, r, :, rep)), msrmnt_rb(k,1:2), zeros(3,3), cov_z);
                else
                    % sensor fusion
                    estm_tg_old = estm_tg(:, target_id);
                    estm_tg_cov_old = estm_tg_cov(:,:,target_id);
                    estm_tg_new = inverse_rb(squeeze(x_true(t, r, :, rep))', msrmnt_rb(k,1:2))';
                    cov_z = [R(r).r_sigma 0; 0 R(r).b_sigma];
                    estm_tg_cov_new = inv_rb_cov(squeeze(x_true(t, r, :, rep)), msrmnt_rb(k,1:2), zeros(3,3), cov_z);
                    
                    info_matrix = inv(estm_tg_cov_old) + inv(squeeze(estm_tg_cov_new));
                    estm_tg_cov_fused = inv(info_matrix);
                    estm_tg_fused = (estm_tg_cov_old\estm_tg_old + squeeze(estm_tg_cov_new)\estm_tg_new)\info_matrix;
                    
                    estm_tg(:, target_id) = estm_tg_fused;
                    estm_tg_cov(:, :, target_id) = estm_tg_cov_fused;
                end

            end
        end
        
        
        
        % Log covariance
        tg_cov_save{t, rep} = estm_tg_cov;
        tg_save{t, rep} = estm_tg;
        

            
            
            
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
                h0.tg_cov = draw_covariances_nx(h0.tg_cov, tmp(:,1:2), tg_cov_save{t,rep},'m');
            end
            title(sprintf('Time Step: %d',t));
           
            drawnow;
            pause(0.125)
            if vid
                
            end
        end
    end
end
% Plot Measurement
