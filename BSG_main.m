clear all;
% Experiment parameters
num_rep = 1;
run_len = 1000;
num_robot = 4;
num_tg = 1;
map_size = 100;

rng(1,'philox');

% Action set for robots
[Vx, Vy] = meshgrid([1, 0, -1],[1, 0, -1]);
ACTION_SET = transpose([Vx(:), Vy(:)]);
ACTION_SET = normalize(ACTION_SET, 1, "norm");
ACTION_SET(isnan(ACTION_SET)) = 0;

% Visibility map
vis_map = init_blank_ndmap([-map_size; -map_size],[map_size; map_size],0.25,'logical');
%vis_map.map = ~vis_map.map;
vis_map_save = cell(run_len,num_rep);

% Initial pose for robots
x_true = zeros(run_len+1, num_robot,3,num_rep); % robots
x_true(1, 1, :, :) = repmat([30;0;0],1,num_rep);
x_true(1, 2, :, :) = repmat([0; 30; pi/2],1,num_rep);
x_true(1, 3, :, :) = repmat([-30; 0; pi],1,num_rep);
x_true(1, 4, :, :) = repmat([0; -30; 3/2*pi],1,num_rep);

% Initial position for targets
tg_true = zeros(3,num_tg,run_len+1,num_rep); % dynamic target
% first two are position, last one is id
tg_true(:,1,1,:) = repmat([80;0;1],1,num_rep);
% tg_true(:,2,1,:) = repmat([0;80;2],1,num_rep);
% tg_true(:,3,1,:) = repmat([-80;0;3],1,num_rep);
% tg_true(:,4,1,:) = repmat([0;-80;4],1,num_rep);


% Measurement History Data
z_d_save = cell(run_len,num_robot,num_rep); % target measurements(range-bearing)
u_save = zeros(run_len,num_robot,2,num_rep); % control

% Esitimate Data
tg_save = cell(run_len,num_rep);
tg_cov_save = cell(run_len,num_rep);
all_tg_cov = zeros(2*num_tg, 2*num_tg, run_len, num_rep);
obj_greedy = zeros(run_len, num_rep);


% Should we get video and image?
vid = false;
viz = true;
planner_name = 'bsg';
vid_name = strcat(strcat('video\four_targets_', planner_name),'_test.mp4');
% planner_name = 'bsg';

for rep = 1:num_rep
    % Create Robots and Planners
    for r = 1:num_robot
        R(r) = robot_nx(x_true(1, r, :, rep));
        P(r) = bsg_planner_nx_v1(num_robot,r, ACTION_SET, run_len, R(r).T, R(r).r_sense,...
            R(r).fov,[R(r).r_sigma;R(r).b_sigma]);

        G(r) = greedy_planner_v2(num_robot, r, ACTION_SET, R(r).T, R(r).r_sense,...
            R(r).fov);
    end
    
    T(1) = target_v1(1, 0.25, tg_true(:,1,1,rep), run_len, 'circle');
%     T(2) = target_v1(2, 0.5, tg_true(:,2,1,rep), run_len, 'random');
%     T(3) = target_v1(3, 0.5, tg_true(:,3,1,rep), run_len, 'random');
%     T(4) = target_v1(4, 0.5, tg_true(:,4,1,rep), run_len, 'random');
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
        h0.tg = [];

        for kk = 1:num_tg
            h0.tg(kk) = draw_pose_nx([], tg_true(:,kk,1,rep),'r',2.2);
        end
        title(sprintf('Time Step: %d',0));
        xlabel('x [m]','FontSize',14);
        ylabel('y [m]','FontSize',14);
        drawnow;
        if vid
            writerObj = VideoWriter(vid_name, 'MPEG-4');
            writerObj.FrameRate = 25;
            open(writerObj);
        end
    end

    % Sense -> Log Measurements -> Plan Moves -> Move Targets -> Move Robots
    for t = 1:run_len

        % Move Targets and get targets' positions at t
        for kk = 1:num_tg
            T(kk).move(t, squeeze(x_true(t, :, :, rep)));
            tg_true(:, kk, t, rep) = T(kk).get_x(t)';
        end

        % Sense
        for r = 1:num_robot
            % targets
            z_d_save{t, r, rep} = R(r).sense(tg_true(:, :, t, rep)');
        end
        
        % Log Mearsurement
        % Key is robot id, Value is a collection of target ids
        target_map = containers.Map('KeyType','double','ValueType','any'); 
        for r = 1:num_robot
            Z_d = z_d_save{t, r, rep};
            target_map(r) = Z_d;
        end
        estm_tg = zeros(2, num_tg);
        estm_tg_cov = zeros(2, 2, num_tg);
        detected = false(1, num_tg);
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
                    detected(target_id) = true;
                else
                    % sensor fusion
                    estm_tg_old = estm_tg(:, target_id);
                    estm_tg_cov_old = estm_tg_cov(:,:,target_id);
                    estm_tg_new = inverse_rb(squeeze(x_true(t, r, :, rep))', msrmnt_rb(k,1:2))';
                    cov_z = [R(r).r_sigma 0; 0 R(r).b_sigma];
                    estm_tg_cov_new = inv_rb_cov(squeeze(x_true(t, r, :, rep)), msrmnt_rb(k,1:2), zeros(3,3), cov_z);
                    
                    info_matrix = inv(estm_tg_cov_old) + inv(squeeze(estm_tg_cov_new));
                    estm_tg_cov_fused = inv(info_matrix);
                    estm_tg_fused = info_matrix\(estm_tg_cov_old\estm_tg_old + squeeze(estm_tg_cov_new)\estm_tg_new);
                    
                    estm_tg(:, target_id) = estm_tg_fused;
                    estm_tg_cov(:, :, target_id) = estm_tg_cov_fused;
                end
            end
        end
              
        % Log covariance
        tg_cov_save{t, rep} = estm_tg_cov(:,:,detected);
        tg_save{t, rep} = estm_tg(:, detected);

        % assign target with zeros obsevation with big covariance

        for kk = 1:num_tg
            if ~detected(kk)
                estm_tg_cov( :, :, kk) = inv_rb_cov([0;0;0], [map_size*2*sqrt(2) 3], zeros(3,3), cov_z);
            end
            all_tg_cov(kk*2-1:kk*2, kk*2-1:kk*2, t, rep) = estm_tg_cov(:, :, kk);
        end


        % At every time step t, first compute objective function using the robots'
        % positions at t (planned at t-1) and the environment at t
        % only detected targets can be considered.
        if strcmp(planner_name, 'greedy')
            obj_greedy(t, rep) = objective_function(squeeze(x_true(t, :, :, rep))', tg_true(1:2,detected, t, rep), R(1).r_sense, R(1).fov);
        end
        % Plan Moves -> compute u_save(t, r, :, rep)
        % both BSG and Greedy only know targets' positions at t
        prev_robot_states = zeros(3, 0);
        for r = 1:num_robot
            if strcmp(planner_name, 'greedy')
                % Greedy: select actions based on targets' positions at t-1,
                % so for Greedy, targets should move to positions at t
                % after Greedy selects actions 
                % TODO: for Greedy, we need to let targets move after Greedy selects actions 
                [next_action_idx, next_state] = G(r).greedy_action(t, squeeze(x_true(t, r, :, rep)), tg_true(1:2, detected, t, rep), prev_robot_states, R(r).r_sense, R(r).fov);

                % move robot
                R(r).set_x(next_state);
                x_true(t,r,:,rep) = next_state;

                % prepare for planning for next robot
                prev_robot_states = [prev_robot_states next_state];

                u_save(t, r, :, rep) = ACTION_SET(:, next_action_idx);
            else
                % BSG: sample actions from p(t) that is based on targets'
                % positions from 1 to t-1
                P(r).update_action_prob_dist(t);
                P(r).selected_action_index(t) = discretesample(P(r).action_prob_dist(t,:), 1);
                u_save(t, r, :, rep) = ACTION_SET(:, P(r).selected_action_index(t));
            end            
        end
        

        
        if strcmp(planner_name, 'bsg')
            % BSG: update experts after selecting actions
            prev_robot_states = zeros(3, 0);
            for r = 1:num_robot

                % previous objective function
                prev_obj_BSG = objective_function(prev_robot_states, tg_true(1:2,:, t, rep), R(r).r_sense, R(r).fov);

                % this robot moves
                R(r).move(squeeze(u_save(t, r, :, rep)));
                x_true(t,r,:,rep) = R(r).get_x();
                prev_robot_states = [prev_robot_states R(r).get_x()];

                % current objective function
                curr_obj_BSG = objective_function(prev_robot_states, tg_true(1:2,:, t, rep), R(r).r_sense, R(r).fov);

                % compute normalized reward, then loss
                reward = (curr_obj_BSG - prev_obj_BSG) / (0 - prev_obj_BSG);
                if reward < 0 || reward > 1
                    error("wrong reward");
                end
                loss = 1 - reward;
                P(r).loss(t, P(r).selected_action_index(t)) = loss;

                % update experts
                P(r).update_experts(t);
            end
        end
       
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
                %tmp
                h0.tg_cov = draw_covariances_nx(h0.tg_cov, tmp(1:2,:), tg_cov_save{t,rep},'m');
            end

            for kk = 1 : num_tg
                h0.tg(kk) = draw_pose_nx(h0.tg(kk), tg_true(:,kk,t,rep),'r',2.2);
            end
            title(sprintf('Time Step: %d',t));
           
            drawnow;
            %pause(0.125)
            if vid
                currFrame = getframe(gcf);
                writeVideo(writerObj, currFrame);
            end
        end
    end
    if viz && vid
        close(writerObj);
    end
end

% Plot Measurement
repToShow = 1;
total_cost = zeros(run_len, num_rep);

for rep = 1 : num_rep
    for t = 1 : run_len
        total_cost(t, rep) = gaussian_entropy_nx(all_tg_cov(:,:, t, rep));
    end
end
fnt_sz = 14;
figure('Color',[1 1 1],'Position',[200 200 500 200]);
if num_rep == 1
    plot(1:run_len,mean(total_cost, 2),'b-','linewidth',2);
else
    shadedErrorBar(1:t, mean(total_cost', 1), std(total_cost'), 'lineprops','g')
end
ylabel({'Target Entropy [nats]'},'FontSize',fnt_sz);
xlabel('Time Steps','FontSize',fnt_sz);
set(gca,'fontsize',fnt_sz);
xlim([0,run_len]);
ylim([-5,15]);
set(gca,'YTick',[-5 0 5 10 15]);
title(planner_name);