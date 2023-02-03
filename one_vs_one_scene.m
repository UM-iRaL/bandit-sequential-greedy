%% non-adversarial single robot vs. single target battle.
clear all;
close all;
% Should we get video and image?
vid = false;
viz = true;
planner_name = 'bsg';
vid_name = strcat(strcat('video\one_vs_one_', planner_name),'_test.mp4');
mode = 'analysis';
% mode = 'experiment';

% Experiment parameters
Horizon = 100;
num_rep = 100;
run_len = 2000;
dT = Horizon / run_len;
num_robot = 1;
num_tg = 1;
map_size = 100;
rng(1,'philox');

% Action set for robots
% [Vx, Vy] = meshgrid([1, 0, -1],[1, 0, -1]);
% ACTION_SET = transpose([Vx(:), Vy(:)]);
% ACTION_SET = normalize(ACTION_SET, 1, "norm");
% ACTION_SET(isnan(ACTION_SET)) = 0;
directions = [0:5] * pi/3;
v_robot = 5;
v_tg = 4;
ACTION_SET = v_robot * [cos(directions); sin(directions)];

% Visibility map
vis_map = init_blank_ndmap([-map_size; -map_size],[map_size; map_size],0.25,'logical');
%vis_map.map = ~vis_map.map;
vis_map_save = cell(run_len,num_rep);

% Initial pose for robots
x_true = zeros(run_len+1, num_robot,3,num_rep); % robots
x_true(1, 1, :, :) = repmat([30;0;0],1,num_rep);


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
estm_tg_save = cell(run_len,num_rep);
estm_tg_cov_save = cell(run_len,num_rep);
all_tg_cov = zeros(2*num_tg, 2*num_tg, run_len, num_rep);
reward = zeros(num_robot, run_len, num_rep);
min_dist = zeros(num_tg, run_len, num_rep);


% planner_name = 'bsg';

for rep = 1:num_rep
    if strcmp(mode, 'analysis')
        viz = false;
        vid = false;
        if rep <= num_rep / 2
            planner_name = 'bsg';
        else
            planner_name = 'greedy';
        end
    end
    % Create Robots and Planners
    for r = 1:num_robot
        R(r) = robot_nx(x_true(1, r, :, rep), 100, deg2rad(94), dT);
        P(r) = bsg_planner_nx_v1(num_robot,r, ACTION_SET, run_len, R(r).T, R(r).r_sense,...
            R(r).fov,[R(r).r_sigma;R(r).b_sigma]);

        G(r) = greedy_planner_v2(num_robot, r, ACTION_SET, R(r).T, R(r).r_sense,...
            R(r).fov);
    end
    
    T(1) = target_v1(1, v_tg, tg_true(:,1,1,rep), run_len, 'circle', dT);
    % Visualization
    if viz
        figure('Color',[1 1 1],'Position',[100,277,1200,800]);
        hold on;
        h0.viz = imagesc([vis_map.pos{1}(1);vis_map.pos{1}(end)],...
            [vis_map.pos{2}(1);vis_map.pos{2}(end)],vis_map.map.');
        cbone = bone; colormap(cbone(end:-1:(end-30),:));
              
        axis([-map_size,map_size,-map_size,map_size]);
        for r = 1:num_robot
            h0.rob(r) = draw_pose_nx([],permute(x_true(1,r,:,rep),[3 2 1]),'b',2.2);
            h0.fov(r) = draw_fov_nx([],permute(x_true(1,r,:,rep),[3 2 1]),R(r).fov,R(r).r_sense, 'b');
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
            writerObj.FrameRate = 40;
            open(writerObj);
        end
    end
    viz = false;
    % Sense -> Log Measurements -> Plan Moves -> Move Targets -> Move Robots
    for t = 1:run_len
        
        if t==run_len-1
            if strcmp(mode, 'experiment')
                viz = true;
            end
        end
        % Move Targets and get targets' positions at t
        if t > 1
            for kk = 1:num_tg
                T(kk).move(t-1, reshape(squeeze(x_true(t-1, :, :, rep)), num_robot,[]));
                tg_true(:, kk, t, rep) = T(kk).get_position(t)';
            end
        end
        % Plan Moves -> compute u_save(t, r, :, rep)
        % both BSG and Greedy only know targets' positions at t
        prev_robot_states = zeros(3, 0);
        prev_r_senses = zeros(1, 0);
        prev_fovs = zeros(1, 0);
        prev_robot_states = zeros(3, 0);
        for r = 1:num_robot
            if strcmp(planner_name, 'greedy')
                if t > 1
                    % Greedy: select actions based on targets' positions at t-1,
                    % so for Greedy, targets should move to positions at t
                    % after Greedy selects actions
                    % TODO: for Greedy, we need to let targets move after Greedy selects actions
                    prev_r_senses = [prev_r_senses R(r).r_sense];
                    prev_fovs = [prev_fovs R(r).fov];
                    [next_action_idx, next_state] = G(r).greedy_action(t, squeeze(x_true(t-1, r, :, rep)), estm_tg_save{t-1, rep}, prev_robot_states, prev_r_senses, prev_fovs);

                    % move robot
%                     R(r).set_x(next_state);
%                     x_true(t,r,:,rep) = next_state;

                    % prepare for planning for next robot
                    prev_robot_states = [prev_robot_states next_state];

                    u_save(t, r, :, rep) = ACTION_SET(:, next_action_idx);
                else
                    num_action = size(ACTION_SET, 2);
                    prob_dist = 1/num_action * ones(num_action, 1);
                    next_action_idx = discretesample(prob_dist, 1);
                    u_save(t, r, :, rep) = ACTION_SET(:, next_action_idx);
                end
           else
                % BSG: sample actions from p(t) that is based on targets'
                % positions from 1 to t-1
                P(r).update_action_prob_dist(t);
                P(r).selected_action_index(t) = discretesample(P(r).action_prob_dist(t,:), 1);
                u_save(t, r, :, rep) = ACTION_SET(:, P(r).selected_action_index(t));
            end            
        end
        % Move Robots
        for r = 1:num_robot
            R(r).move(squeeze(u_save(t, r, :, rep)));
            x_true(t,r,:,rep) = R(r).get_x();
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
                    estm_tg_1 = estm_tg(:, target_id);
                    estm_tg_cov_1 = estm_tg_cov(:,:,target_id);
                    estm_tg_2 = inverse_rb(squeeze(x_true(t, r, :, rep))', msrmnt_rb(k,1:2))';
                    cov_z = [R(r).r_sigma 0; 0 R(r).b_sigma];
                    estm_tg_cov_2 = squeeze(inv_rb_cov(squeeze(x_true(t, r, :, rep)), msrmnt_rb(k,1:2), zeros(3,3), cov_z));
                    beta = (estm_tg_cov_1 + estm_tg_cov_2)\estm_tg_cov_2;
                    estm_tg(:, target_id) = beta*estm_tg_1 + (eye(2) - beta)*estm_tg_2;
                    estm_tg_cov(:, :, target_id) = beta*estm_tg_cov_1*beta' + (eye(2)-beta)*estm_tg_cov_2*(eye(2)-beta)';
                end
            end
        end
              
        % Log covariance
        estm_tg_cov_save{t, rep} = estm_tg_cov(:,:,detected);
        estm_tg_save{t, rep} = estm_tg(:, detected);
        estm_tg =  estm_tg(:, detected);
        % assign target with zeros obsevation with big covariance

        for kk = 1:num_tg
            if ~detected(kk)
                cov_z = [R(r).r_sigma 0; 0 R(r).b_sigma];             
                estm_tg_cov( :, :, kk) = 1e6*eye(2);
            end
            all_tg_cov(kk*2-1:kk*2, kk*2-1:kk*2, t, rep) = estm_tg_cov(:, :, kk);
        end


        % At every time step t, first compute objective function using the robots'
        % positions at t (planned at t-1) and the environment at t
        % only detected targets can be considered.
        r_senses = zeros(1, num_robot);
        fovs = zeros(1, num_robot);
        for i = 1:num_robot
            r_senses(i) = R(r).r_sense;
            fovs(i) = R(r).fov;
        end
                
        if strcmp(planner_name, 'bsg')
            % BSG: update experts after selecting actions
            prev_robot_states = zeros(3, 0);
            prev_r_senses = zeros(1, 0);
            prev_fovs = zeros(1, 0);

            r_v = 1:num_robot;
            itr_order = r_v(randperm(length(r_v)));
            for r = itr_order
                if size(estm_tg_save{t, rep}, 2) ~= 0
                    % previous objective function
                    prev_obj_BSG = objective_function(prev_robot_states, estm_tg_save{t, rep}, prev_r_senses, prev_fovs);
                    
                    % now consider new robot position
                    prev_robot_states = [prev_robot_states R(r).get_x()];
                    prev_r_senses = [prev_r_senses R(r).r_sense];
                    prev_fovs = [prev_fovs R(r).fov];

                    % current objective function
                    curr_obj_BSG = objective_function(prev_robot_states, estm_tg_save{t, rep}, prev_r_senses, prev_fovs);

                    % compute normalized reward, then loss
                    reward(r, t, rep) = (curr_obj_BSG - prev_obj_BSG) / (0 - prev_obj_BSG);
%                     reward(r, t, rep) = (curr_obj_BSG - prev_obj_BSG) / (prev_obj_BSG/2 - prev_obj_BSG);

                    if reward(r, t, rep) < 0 || reward(r, t, rep) > 1                       
                        error("wrong reward");
                    end
                    loss = 1 - reward(r, t, rep);
                    P(r).loss(t, P(r).selected_action_index(t)) = loss;
                else
                    P(r).loss(t, P(r).selected_action_index(t)) = 1;
                end

                % update experts
                P(r).update_experts(t);
            end
        end
       
        % Visualization
        if viz
            set(h0.viz,'cdata',vis_map.map.');

            h0.y = draw_traj_nx([],permute(tg_true(:,:,1:t,rep),[3 1 2 4]),'r:');

            for r = 1:num_robot
                h0.rob(r) = draw_pose_nx(h0.rob(r),permute(x_true(t,r,:,rep),[3 2 1]),'b',2.2);
                h0.fov(r) = draw_fov_nx(h0.fov(r),permute(x_true(t,r,:,rep),[3 2 1]),R(r).fov,R(r).r_sense);
                
                h0.r_traj(r) = draw_traj_nx([],permute(x_true(1:t,r,:,rep),[1 3 2 4]),'b:');
            end
            tmp = estm_tg_save{t, rep};
            if ~isempty(tmp)
                %tmp
                h0.tg_cov = draw_covariances_nx(h0.tg_cov, tmp(1:2,:), estm_tg_cov_save{t,rep},'m');
            else
                delete(h0.tg_cov);
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
    for kk = 1:num_tg
        %min_dist(kk, 1:end-1, rep) = T(kk).all_min_dist(:)';
        for t = 1:run_len
            min_dist(kk, t, rep) = T(kk).min_dist_to_robots(t, squeeze(x_true(t,:,:,rep)));
        end
    end
    if viz && vid
        close(writerObj);
    end
end

if strcmp(mode, 'analysis')
    % Plot Measurement
    repToShow = 1;
    total_cost_bsg = zeros(run_len, num_rep/2);
    total_cost_greedy = zeros(run_len, num_rep/2);

    for rep = 1 : num_rep
        for t = 1 : run_len
            if rep <= num_rep/2
                total_cost_bsg(t, rep) = gaussian_entropy_nx(all_tg_cov(:,:, t, rep));
            else
                total_cost_greedy(t, rep - num_rep/2) = gaussian_entropy_nx(all_tg_cov(:,:, t, rep));
            end
        end
    end
    fnt_sz = 10;
    figure('Color',[1 1 1],'Position',[200 200 500 200]);
    if num_rep == 1
        plot(1:run_len,mean(total_cost, 2),'b-','linewidth',2);
    else
        h1 = shadedErrorBar(1:t, mean(total_cost_bsg', 1), std(total_cost_bsg'), 'lineprops',{'Color',"#77AC30", 'LineWidth', 1});
        h2 = shadedErrorBar(1:t, mean(total_cost_greedy', 1), std(total_cost_greedy'), 'lineprops',{'Color',"#D95319", 'LineWidth', 1});
    end
    legend([h1.mainLine h2.mainLine], 'BSG', 'SG');
    ylabel({'Target Entropy [nats]'},'FontSize',fnt_sz);
    xlabel('Time Steps','FontSize',fnt_sz);
    set(gca,'fontsize',fnt_sz);
    xlim([0,run_len]);
    ylim([-5,30]);
    set(gca,'YTick',[-5 0 5 10 15 20 25 30]);

    total_obj_bsg = zeros(run_len, num_rep/2);
    total_obj_greedy = zeros(run_len, num_rep/2);
    r_senses = zeros(1, num_robot);
    fovs = zeros(1, num_robot);
    for i = 1:num_robot
        r_senses(i) = R(r).r_sense;
        fovs(i) = R(r).fov;
    end

    for rep = 1 : num_rep
        for t = 1 : run_len
            if rep <= num_rep/2
                total_obj_bsg(t, rep) = objective_function(squeeze(x_true(t, :, :, rep)), tg_true(1:2,:,t,rep), r_senses, fovs);
            else
                total_obj_greedy(t, rep - num_rep/2) = objective_function(squeeze(x_true(t, :, :, rep)), tg_true(1:2,:,t,rep), r_senses, fovs);
            end
        end
    end
    figure('Color',[1 1 1],'Position',[700 200 500 200]);
    if num_rep == 1
        plot(1:run_len,mean(total_cost, 2),'b-','linewidth',2);
    else
        h3 = shadedErrorBar(dT*[1:t], mean(total_obj_bsg', 1), std(total_obj_bsg'), 'lineprops',{'Color',"#0072BD", 'LineWidth', 1});
        h4 = shadedErrorBar(dT*[1:t], mean(total_obj_greedy', 1), std(total_obj_greedy'), 'lineprops',{'Color',"#D95319", 'LineWidth', 1});
    end
    legend([h3.mainLine h4.mainLine], 'BSG', 'SG');
    ylabel({'Objective Function'},'FontSize',fnt_sz);
    xlabel('Time [s]','FontSize',fnt_sz);
    set(gca,'fontsize',fnt_sz);
    xlim([0,run_len*dT]);
    ylim([-Inf, 0]);
    %title(planner_name);


    dist_bsg = zeros(run_len, num_rep/2);
    dist_greedy = zeros(run_len, num_rep/2);
    for rep = 1 : num_rep
        for t = 1 : run_len
            if rep <= num_rep/2
                dist_bsg(t, rep) = sum(min_dist(:,t, rep));
            else
                dist_greedy(t, rep - num_rep/2) = sum(min_dist(:,t, rep));
            end
        end
    end
    figure('Color',[1 1 1],'Position',[1200 200 500 200]);

    h5 = shadedErrorBar(dT*[1:t], mean(dist_bsg', 1), std(dist_bsg'), 'lineprops',{'Color',"#0072BD", 'LineWidth', 1});
    h6 = shadedErrorBar(dT*[1:t], mean(dist_greedy', 1), std(dist_greedy'), 'lineprops',{'Color',"#D95319", 'LineWidth', 1});
    
end