clear all;
close all;
% Experiment parameters
num_rep = 1;
run_len = 1500;
num_robot = 3;
num_tg = 1;
map_size = 150;
rng(1,'philox');
% Should we get video and image?
vid = false;
viz = true;
planner_name = 'bsg';
%planner_name = 'random';
%planner_name = 'bsg';
vid_name = strcat(strcat('video\dd_single_target_random_order_', planner_name),'_test.mp4');
mode = 'experiment';
%mode = 'experiment';
if strcmp(mode, 'analysis')
   num_rep = num_rep*3;
end

% Action set for robots
%[V,W] = meshgrid([1],[0, -1, 1, -2, 2]);
%ACTION_SET = transpose([V(:), W(:)]);
[Vx, Vy] = meshgrid([1, 0, -1],[1, 0, -1]);
ACTION_SET = transpose([Vx(:), Vy(:)]);
% Visibility map
vis_map = init_blank_ndmap([-map_size; -map_size],[map_size; map_size],0.25,'logical');
%vis_map.map = ~vis_map.map;
vis_map_save = cell(run_len,num_rep);

% Initial pose for robots
x_true = zeros(run_len+1, num_robot+2,3,num_rep); % robots
x_true(1, 1, :, :) = repmat([-30;-30;0],1,num_rep);
x_true(1, 2, :, :) = repmat([30; 30; pi/4],1,num_rep);
x_true(1, 3, :, :) = repmat([-30; 0; pi/4],1,num_rep);

% Initial position for targets
tg_true = zeros(3,num_tg,run_len+1,num_rep); % dynamic target
% first two are position, last one is id
tg_true(:,1,1,:) = repmat([120;0;1],1,num_rep);
%tg_true(:,2,1,:) = repmat([40;40;2],1,num_rep);
%tg_true(:,3,1,:) = repmat([-60;0;3],1,num_rep);

% Measurement History Data
z_d_save = cell(run_len,num_robot,num_rep); % target measurements(range-bearing)
u_save = zeros(run_len,num_robot,2,num_rep); % control

% Esitimate Data
tg_save = cell(run_len,num_rep);
tg_cov_save = cell(run_len,num_rep);
all_tg_cov = zeros(2*num_tg, 2*num_tg, run_len, num_rep);

for rep = 1:num_rep
    if strcmp(mode, 'analysis')
        if rep <= num_rep/3
            planner_name = 'greedy';
        elseif rep > num_rep/3 && rep <= 2/3*num_rep
            planner_name = 'bsg';
        else
            planner_name = 'random';
        end
    end

    % Create Robots and Planners
    for r = 1:num_robot
        R(r) = robot_nx(x_true(1, r, :, rep));
        G(r) = greedy_planner_v1(num_robot, r, ACTION_SET, R(r).T, R(r).r_sense,...
            R(r).fov);
        P(r) = bsg_planner_nx_v1(num_robot,r, ACTION_SET, run_len, R(r).T, R(r).r_sense,...
            R(r).fov,[R(r).r_sigma;R(r).b_sigma]);
    end

    % Create Targets.
    T(1) = target_v1(0.5, tg_true(:,1,1,rep), run_len, 'circle');
    %T(2) = target_v1(0.5, tg_true(:,2,1,rep), run_len, 'rect');
    %T(3) = target_v1(0.5, tg_true(:,3,1,rep), run_len, 'random');
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

    % Sense -> Plan Moves -> Move Targets -> Move Robots -> Log
    % Measurements -> Compute Loss
    for t = 1:run_len
        % Sense
        for r = 1:num_robot
            % targets
            z_d_save{t, r, rep} = R(r).sense(tg_true(:, :, t, rep)');
        end

        % Plan Moves
        prev_loss = 1;
        prev_obj_mat = zeros(num_tg, 1);
        for r = 1:num_robot
            % Greedy
            if strcmp(planner_name, 'greedy')
                [loss, obj_mat, action_idx] = G(r).greedy_action(t, r, squeeze(x_true(t, r, :, rep)), tg_true(:, :, t, rep)', prev_loss, prev_obj_mat);
                prev_loss = loss;
                prev_obj_mat = obj_mat;
                u_save(t, r, :, rep) = ACTION_SET(:,action_idx);
            elseif strcmp(planner_name, 'bsg')
            % BSG
                P(r).update_action_prob_dist(t);
                P(r).next_action_index(t) = discretesample(P(r).action_prob_dist(t,:), 1);
                u_save(t, r, :, rep) = ACTION_SET(:, P(r).next_action_index(t));
            elseif strcmp(planner_name, 'random')
                uniform_dist = 1/size(ACTION_SET, 2)*ones(1, size(ACTION_SET, 2));
                next_action_set = discretesample(uniform_dist, 1);
                u_save(t, r, :, rep) = ACTION_SET(:, next_action_set);
            end            
        end

        % Move Targets
        v_tg = 0.25;
        for kk = 1:num_tg
            if kk == 1
                %tg_true(:, kk, t+1, rep) = tg_true(:,kk,t,rep) +  [(t - 50 < 0)*v_tg; 0;0] + ...
                %   [0; (t - 50 > 0)*(t - 100 < 0)*v_tg; 0] + [(t - 100 > 0)*(t - 150 < 0)*(-v_tg); 0; 0] + [0; (t - 150 > 0)*(t - 200 < 0)*(-v_tg); 0];
                T(kk).move(t, squeeze(x_true(t, :, :, rep))');
                tg_true(:, kk, t+1, rep) = T(1).get_x(t+1)';
            elseif kk == 2
                %tg_true(:, kk, t+1, rep) = tg_true(:,kk,t,rep) +  [(t - 50 < 0)*(-v_tg); 0;0] + ...
                 %   [0; (t - 50 > 0)*(t - 100 < 0)*v_tg; 0] + [(t - 100 > 0)*(t - 150 < 0)*(v_tg); 0; 0] + [0; (t - 150 > 0)*(t - 200 < 0)*(-v_tg); 0];
                T(kk).move(t, squeeze(x_true(t, :, :, rep))');
                tg_true(:, kk, t+1, rep) = T(kk).get_x(t+1)';
            else
                %tg_true(:,kk,t+1,rep) = A*tg_true(:,kk,t,rep)+[0.05;0;0] + [chol(W(:,:,kk,t,rep)).'*randn(2,1); 0]; % add Gaussian noise
                T(kk).move(t, squeeze(x_true(t, :, :, rep))');
                tg_true(:, kk, t+1, rep) = T(kk).get_x(t+1)';
            end            
        end
        
        % BSG: update experts after selecting actions
        prev_loss = 1;
        prev_obj_mat = zeros(num_tg, 1);
        r_v = 1:num_robot;
        itr_rank = r_v(randperm(length(r_v)));

        for r = itr_rank
            R(r).move(squeeze(u_save(t, r, :, rep)))%, point_mass_motion_model);
            x_true(t+1,r,:,rep) = R(r).get_x();
            % f(a_i|A_{i-1})
            [loss, obj_mat] = P(r).losses_after_action(t, r, squeeze(x_true(t,r,:,rep)), tg_true(:,:, t+1, rep)', squeeze(u_save(t, r, :, rep)), prev_loss, prev_obj_mat);
            prev_obj_mat = obj_mat;
            prev_loss = loss;
            P(r).update_experts(t);
        end
        
        % Log Mearsurement
            % 1. Maintain a map from robots to detected targets
            % 2. Compute covariance based on map
            
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
                estm_tg_cov( :, :, kk) = inv_rb_cov([0;0;0], [2*500*sqrt(2) 3], zeros(3,3), cov_z);
            end
            all_tg_cov(kk*2-1:kk*2, kk*2-1:kk*2, t, rep) = estm_tg_cov(:, :, kk);
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

if strcmp(mode, 'analysis')
    total_cost_greedy = zeros(run_len, num_rep/3);
    for rep = 1 : num_rep/3
        for t = 1 : run_len
            total_cost_greedy(t, rep) = gaussian_entropy_nx(all_tg_cov(:,:, t, rep));
        end
    end
    total_cost_bsg = zeros(run_len, num_rep/3);
    for rep = num_rep/3+1 : 2/3*num_rep
        for t = 1 : run_len
            total_cost_bsg(t, rep-1/3*num_rep) = gaussian_entropy_nx(all_tg_cov(:,:, t, rep));
        end
    end
    total_cost_random = zeros(run_len, num_rep/3);
    for rep = 2/3*num_rep+1 : num_rep
        for t = 1 : run_len
            total_cost_random(t, rep-2/3*num_rep) = gaussian_entropy_nx(all_tg_cov(:,:, t, rep));
        end
    end
    fnt_sz = 10;
    figure('Color',[1 1 1],'Position',[200 200 500 200]);
    %plot(1:run_len,mean(total_cost, 2),'b-','linewidth',2);
    shadedErrorBar(1:t, mean(total_cost_greedy', 1), std(total_cost_greedy'), 'lineprops','g')
    shadedErrorBar(1:t, mean(total_cost_bsg', 1), std(total_cost_bsg'), 'lineprops','b')
    shadedErrorBar(1:t, mean(total_cost_random', 1), std(total_cost_random'), 'lineprops','r')
    ylabel({'Target Entropy [nats]'},'FontSize',fnt_sz);
    xlabel('Time Steps','FontSize',fnt_sz);
    set(gca,'fontsize',fnt_sz);
    xlim([0,run_len]);
    ylim([-10,10]);
    set(gca,'YTick',[-10 -5 0 5 10]);
else
    repToShow = 1;
    total_cost = zeros(run_len, num_rep);
    for rep = 1 : num_rep
        for t = 1 : run_len
            total_cost(t, rep) = gaussian_entropy_nx(all_tg_cov(:,:, t, rep));
        end
    end

    fnt_sz = 10;
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
    ylim([-10,10]);
    set(gca,'YTick',[-10 -5 0 5 10]);
    title(planner_name);
    %mean_cost = mean(total_cost)
end
