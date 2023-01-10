clear;
close;
num_rep = 1;
run_len = 200;
num_robot = 3;
num_tg = 2; % number of dynamic targets
dim_tg = 1;
v_tg = 0.35;
exp_num = 5;
reset_time = 1000;
% build discrete action set
% ACTION_SET = zeros(2, 4, 3);
% for i = 1:4
%     for j = 1:3
%         ACTION_SET(:,i,j) = [0.25*(i-1);0.25*deg2rad(45*j - 90)];
%     end
% end
% ACTION_SET = reshape(ACTION_SET, 2,[]);
[V,W] = meshgrid([3,1],[0, -1, 1, -2, 2]);
ACTION_SET = transpose([V(:), W(:)]);
viz = true;
vid = true;
% Create environment
E = environment_nx;
load('Emap_saved.mat');
E.set_map(Emap_saved);

% fake landmark
y_exp_save = cell(run_len,num_rep);
% Dynamic properties
tmpW = 0.1*eye(dim_tg);
W = repmat(tmpW,1,1,num_tg,run_len,num_rep);
A = eye(3);

% Visibility map
vis_map = init_blank_ndmap([-E.dev;-E.dev],[E.dev;E.dev],0.25,'logical');
vis_map.map = ~vis_map.map;
vis_map_save = cell(run_len,num_rep);


% Ground Truth Data
y_true = E.get_map();                         % landmark
y_true_cell = meters2cells(y_true(:,1:2),vis_map.min.',vis_map.res);  % convert to cell
y_true_linidx = sub2ind_nx(vis_map.size.',y_true_cell(:,1),y_true_cell(:,2)); % landmark idx
y_true(:,3) = 1;

%y_d_true = zeros(3,num_y_d,run_len+1,num_rep); % dynamic target
tg_true = zeros(3,num_tg,run_len+1,num_rep); % dynamic target
tg_true(:,1,1,:) = repmat([-36;-20;1],1,num_rep);
tg_true(:,2,1,:) = repmat([-15;0;2],1,num_rep);
%tg_true(:,3,1,:) = repmat([5;-20;2],1,num_rep);
%y_d_true(:,3,1,:) = repmat([-5;5],1,num_rep);

x_true = zeros(run_len+1, num_robot,3,num_rep); % robots
%x_true = zeros(run_len+1,5,3,num_rep); % robots
x_true(1,1,:,:) = repmat([0;-20;0],1,num_rep);
x_true(1,2,:,:) = repmat([-40;-20;0],1,num_rep);
%x_true(1,2,:,:) = repmat([-11;-6;0],1,num_rep);
x_true(1,3,:,:) = repmat([-20;0;0],1,num_rep);

% Measurement History Data
z_save = cell(run_len,num_robot,num_rep);   % landmark measurement
z_d_save = cell(run_len,num_robot,num_rep); % target measurement
u_save = zeros(run_len,num_robot,2,num_rep);
G_tg_idx = [];
% Estimates Data
y_save = cell(run_len,num_rep);
y_cov_save = cell(run_len,num_rep);
tg_save = cell(run_len,num_rep);
tg_cov_save = cell(run_len,num_rep);
xy_cholimat_save = cell(run_len,num_rep);

x_save = zeros(run_len+1,num_robot,3,num_rep);
x_cov_save = zeros(run_len,num_robot,3,3,num_rep);

% Experiment
u_const = [0.2;0.1];

for rep = 1:num_rep
    
    
    % Create Robots
    for r = 1:num_robot
        R(r) = robot_nx(x_true(1,r,:,rep));
        P(r) = osg_planner_nx_v1(num_robot,r, ACTION_SET, run_len, R(r).T, R(r).r_sense,...
            R(r).fov,[R(r).r_sigma;R(r).b_sigma]);
    end
    inv_sqrt_cov_z = inv(diag([R(1).r_sigma; R(1).b_sigma]));
    inv_sqrt_cov_u = chol(40*speye(3));      % SRIF
    % Prior on x
    estm_x = cell(num_robot,1);
    estm_x_cov = zeros(3,3,num_robot);
    for r = 1:num_robot
        estm_x{r} = R(r).get_x(); %[2;2;1].*randn(3,1);
        estm_x_cov(:,:,r) = diag([0.001,0.001,0.001]); %diag([2;2;1].^2);
    end
    
    % Save
    for r = 1:num_robot
        x_save(1,r,:,rep) = estm_x{r};
        x_cov_save(1,r,:,:,rep) = estm_x_cov(:,:,r);
    end
    
    % Slam graph
    G = graph_multirobotslam(num_robot);
    for r = 1:num_robot
        G.add_prior_x(r,1,estm_x{r},chol(estm_x_cov(:,:,r)\speye(3)));
        G.append_estimate_x(r,estm_x{r});
    end
    
    if viz
        figure('Color',[1 1 1],'Position',[100,277,1200,800]);
        hold on;
        h0.viz = imagesc([vis_map.pos{1}(1);vis_map.pos{1}(end)],...
            [vis_map.pos{2}(1);vis_map.pos{2}(end)],vis_map.map.');
        cbone = bone; colormap(cbone(end:-1:(end-30),:));
        
        h0.map = drawEnv(y_true(~vis_map.map(y_true_linidx),:),1);
        
        axis([-E.dev,E.dev,-E.dev,E.dev]);
        set(gca,'fontsize',14);
        for r = 1:num_robot
            h0.rob(r) = draw_pose_nx([],permute(x_true(1,r,:,rep),[3 2 1]),'g',2.2);
            h0.fov(r) = draw_fov_nx([],permute(x_true(1,r,:,rep),[3 2 1]),R(r).fov,R(r).r_sense);
        end
        % Draw Estimates
        h0.xe = draw_traj_nx([],permute(x_save(1,:,:,rep),[1 3 2]),'r:');
        h0.ye = [];
        
        % Draw uncertainties
        h0.xcov = draw_covariances_nx([],permute(x_save(1,:,1:2,rep),[3 2 1]),...
                permute(x_cov_save(1,:,1:2,1:2,rep),[3 4 2 1 5]));
        h0.ycov = [];
        h0.tg_cov = [];
        title(sprintf('Time Step: %d',0));
        xlabel('x [m]','FontSize',14);
        ylabel('y [m]','FontSize',14);
        drawnow;
        if vid
              writerObj = VideoWriter('three_robots.avi');
              writerObj.FrameRate = 25;
              open(writerObj);
        end
    end
    
    % map true landmark ids to consecutive ids for slam
    if( exist('landmark_map','var') )
        delete(landmark_map);
    end
    landmark_map = containers.Map('KeyType','double','ValueType','double');
    reverse_landmark_map = [];

    % Main Loop
    for t = 1:run_len
        
        %% Sense
        for r = 1:num_robot
            % landmark 
            z_save{t,r,rep} = R(r).sense(y_true);
            % targets
            z_d_save{t,r, rep} = R(r).sense(tg_true(:,:, t, rep)');
        end
        
        % every time updating target_map, in case one single target seen by
        % multiple robots
        target_map = containers.Map('KeyType','double','ValueType','double');
        
        %% Update SLAM Graph
        curr_t = mod(t-1,reset_time)+1;
        for r = 1:num_robot
            % for static landmark
            Z = z_save{t,r,rep};
            for k = 1:size(Z,1)
                true_lm_id = Z(k,end);
                % check if seen before
                if( ~landmark_map.isKey(true_lm_id) )
                    landmark_map( true_lm_id ) = G.n_y+1;
                    reverse_landmark_map( G.n_y+1 ) = true_lm_id;
                    G.append_estimate_y(inverse_rb(squeeze(x_save(t,r,:,rep)).', Z(k,1:2)).');
                end
                G.add_meas_rangebearing(r,curr_t,landmark_map(true_lm_id),Z(k,1:2).',inv_sqrt_cov_z);
            end
            % for dynamic landmark
            Z_d = z_d_save{t,r, rep};
            for k = 1:size(Z_d, 1)
                
                % if at current time, dynamic target is seen by multiple
                % robots, 
                true_tg_id = Z_d(k, end);
                if(~target_map.isKey(true_tg_id))
                    target_map(true_tg_id) = G.n_y+1;
                    G.append_estimate_y(inverse_rb(squeeze(x_save(t,r,:,rep)).', Z_d(k,1:2)).');
                end
                G.add_meas_rangebearing(r,curr_t,target_map(true_tg_id),Z_d(k,1:2).',inv_sqrt_cov_z);
            end
        end
        tg_det = target_map.Count; 
        if tg_det ~= 0
            % store target index in the graph
            tg_idx = cell2mat(values(target_map));
            tg_idx = tg_idx(:);          
            G_tg_idx = [G_tg_idx;tg_idx];
        end
        y_idx = setdiff(1:G.n_y, G_tg_idx);
        %% Optimize SLAM Graph
        G.solve(0.01,100);
        estm_x = G.get_estimate_x();
        estm_y = G.get_estimate_y();        % 2 x num_y
        estm_tg = estm_y(:, tg_idx);
        estm_y = estm_y(:, y_idx);
        [estm_x_cov, estm_y_cov, estm_xy_cov] = G.get_cov();
        estm_tg_cov = estm_y_cov(:, :, tg_idx); 
        estm_y_cov = estm_y_cov(:,:,y_idx);

        %% Save estimates
        idx0 = (t+1-size(estm_x{1},2)):t;
        for r = 1:num_robot
            x_save(idx0,r,:,rep) = transpose(estm_x{r});
            x_cov_save(t,r,:,:,rep) = estm_x_cov(:,:,r);
        end
        y_save{t,rep} = transpose([estm_y;3*ones(1,size(estm_y,2))]);
        y_cov_save{t,rep} = estm_y_cov;
        if tg_det ~= 0
            tg_save{t,rep} = transpose([estm_tg;3*ones(1,size(estm_tg,2))]);
            tg_cov_save{t,rep} = estm_tg_cov;
        end
        % y_d_cov_save{t, rep}(end, end, 1) = 0;
        %% Determine exploration landmarks, spread exploration landmarks around targets.
        exp_y = zeros(num_tg*exp_num, 2);
        exp_r = 2.5;
        exp_theta = 360 / exp_num;
        for i = 1: num_tg
            % spread around targets and check if the landmarks is valid.
            % TODO: if we want to round the exp_y
            tg_pos = tg_true(1:2, i, t, rep);
            for j = 1:exp_num
                exp_y((i-1)*exp_num+j, 1) = tg_pos(1) + exp_r*cos(deg2rad((j-1)*exp_theta)); 
                exp_y((i-1)*exp_num+j, 2) = tg_pos(2) + exp_r*sin(deg2rad((j-1)*exp_theta)); 
            end
        end
        y_exp_save{t,rep} = [exp_y,ones(size(exp_y,1),1)];

        %% Determine cholimat btw robots and targets.
        match_y = cell(num_robot,1);
        plan_y = cell(num_robot,1);
        plan_y_cov = cell(num_robot,1);
        if tg_det ~= 0
            for r = 1:num_robot
                srange = 3*max(ACTION_SET(1,:))*R(r).T*12;                      %searching range, could self-defined
                match = rangesearch(transpose(estm_y),estm_x{r}(1:2,end).',srange);
                match = match{1};
                match_y{r} = tg_idx;
                plan_y{r} = estm_tg;
                plan_y_cov{r} = estm_tg_cov;
            end
        end
        [jC, plan_cholimat, jmC] = G.get_cholimat(mat2cell(G.n_x,ones(size(G.n_x))),match_y);
        xy_cholimat_save{t,rep} = jmC{end};  %jc is cholimat of information matrix of robots and targets

         
        fixed_x0 = cell(num_robot,1); fixed_u = cell(num_robot,1);
        for r = 1:num_robot

            %u_save(t,r,:,rep) = P(r).plan(estm_x{r}(:,end),estm_x_cov(:,:,r),plan_y{r},plan_y_cov{r},...
            %    plan_cholimat{r},exp_y,eps,del,Tmax,replan_time);
            P(r).get_action_prob_dist(t);
            P(1).action_prob_dist(t,:)
            next_action_idx = discretesample(P(r).action_prob_dist(t,:), 1);
            u_save(t,r,:,rep) = ACTION_SET(:, next_action_idx);
            fixed_x0{r} = estm_x{r}(:,end);
            fixed_u{r} = ACTION_SET(:, next_action_idx);
            
            %u_plan_save{t,r,rep} = P(r).u_plan;
            %attractors{r} = P(r).att_save;
        end     
        %% Move Robots        
        for r = 1:num_robot
            R(r).move(squeeze(u_save(t,r,:,rep)));
        end
        %P(1).action_prob_dist(t,:)
        %% Move Dynamic targets
        for kk = 1:num_tg
            if kk == 1
                tg_true(:, kk, t+1, rep) = tg_true(:,kk,t,rep) +  [(t - 50 < 0)*v_tg; 0;0] + ...
                    [0; (t - 50 > 0)*(t - 100 < 0)*v_tg; 0] + [(t - 100 > 0)*(t - 150 < 0)*(-v_tg); 0; 0] + [0; (t - 150 > 0)*(t - 200 < 0)*(-v_tg); 0];
            elseif kk == 2
                tg_true(:, kk, t+1, rep) = tg_true(:,kk,t,rep) +  [(t - 50 < 0)*(-v_tg); 0;0] + ...
                    [0; (t - 50 > 0)*(t - 100 < 0)*v_tg; 0] + [(t - 100 > 0)*(t - 150 < 0)*(v_tg); 0; 0] + [0; (t - 150 > 0)*(t - 200 < 0)*(-v_tg); 0];
            else
            tg_true(:,kk,t+1,rep) = A*tg_true(:,kk,t,rep)+[0.05;0;0] + [chol(W(:,:,kk,t,rep)).'*randn(2,1); 0]; % add Gaussian noise
            end
        end
        %% Get f_t(empty)  
        % Step 1: get f_t(empty) -> xy_cholimat_save{t,rep}
        if t > 1
            R_x_sqrt = xy_cholimat_save{t-1,rep};
            % exclude information matrix of targets' position
            R_x = R_x_sqrt(1:num_robot*3, 1:num_robot*3);
            
            I_pre = transpose(R_x)*R_x; %information matrix for robots' poses.. TODO: determine if it is..
            tg_cur = tg_save{t, rep}; %num_det x 3
            x_pre = squeeze(x_true(t-1, :, :, rep)); %num_x x 3
            num_det = size(tg_cur,1); % number of detections given target of current time and robots of last time step.
            if num_det ~= 0
                detectable = detectable2D(tg_cur(:,1:2),x_pre,R(1).r_sense,R(1).fov,true); % num_y x num_x
                det_y_idx = sum(detectable,2) > 0;
                num_detectable = sum(sum(detectable,2) > 0); % num_y x 1
            else
                num_detectable = 0;
            end
            if num_detectable ~= 0
                I_tg = zeros(num_detectable*2, num_detectable*2);
                %TODO: change to estimates
                
                
                H_xy = H_rb_xy(squeeze(x_pre), tg_cur(det_y_idx,1:2), R(1).r_sense,R(1).fov); % add exp_y
                inv_z = inv_sqrt_cov_z*inv_sqrt_cov_z;
                % for each detectable pairs, I_empty + H_xy'*cov_z-1*H_xy;
                
                sep_pair = 3+2*num_detectable;
                I_empty = blkdiag(I_pre, I_tg);
                for r = 1:num_robot
                    for i = 1:num_detectable
                        H = zeros(2,3*num_robot+2*num_detectable); % H=[0_{1:r-1},H_x(r),0_{r+1:num_robot},0_{1:i-1}, H_y(i),0_{i+1:tg_det}]
                        H(1:2, (r-1)*3+1:r*3) = H_xy(2*(i-1)+1:2*(i-1)+2,sep_pair*(r-1)+1:sep_pair*(r-1)+3);
                        H(1:2, 3*num_robot+2*(i-1)+1:3*num_robot+2*(i-1)+2) = H_xy(2*(i-1)+1:2*(i-1)+2,sep_pair*(r-1)+4+(i-1)*2:sep_pair*(r-1)+5+(i-1)*2);
                        I_empty = I_empty + transpose(H)*inv_z*H; 
                    end
                end
                
                
            else
                I_empty = I_pre;
            end
            f_empty = -logdet_nx(I_empty);
            %R_empty = qr(I_empty);
            R_init = chol(I_empty); %
            
            % get current location
        end
        %% Update Loss
        for r = 1:num_robot
            % use true data for experimenting
            if t > 1

                    P(r).losses_after_action(t, r, squeeze(x_save(t, r, :, rep)), x_cov_save(t, r, :, rep), tg_save{t, rep}, R_init, exp_y,...
                    fixed_x0, fixed_u);

                %P(r).get_losses(t, rep, tg_true, x_true, r);
            end
            P(r).update_experts(t);
        end
        % Save data
        for r = 1:num_robot
            x_true(t+1,r,:,rep) = R(r).get_x();
            %R(r).get_x()
            x_save(t+1,r,:,rep) = dd_motion_model(squeeze(x_save(t,r,:,rep)),...
                squeeze(u_save(t,r,:,rep)),R(r).T);
        end
        
        %% Add Odometry to SLAM
        for r = 1:num_robot
            G.add_odom_relpose(r,t,t+1,dd_motion_model(zeros(3,1),...
                squeeze(u_save(t,r,:,rep)),R(r).T),inv_sqrt_cov_u);
            G.append_estimate_x(r,squeeze(x_save(t+1,r,:,rep)));
        end
        
        
        %% Remove visibility
        for r = 1:num_robot
            [im,jm] = find(vis_map.map);
            if( ~isempty(im) )
                vis_y = cells2meters([im,jm],vis_map.min.',vis_map.res);
                det_y = detectable2D(vis_y,R(r).get_x().',R(r).r_sense,R(r).fov); % num_y x 1
                det_im = im(det_y);
                det_jm = jm(det_y);
                lindix = sub2ind_nx(vis_map.size(1),det_im,det_jm);
                vis_map.map(lindix) = false;
            end
        end
        vis_map_save{t,rep} = vis_map;
        
        %% draw
        if viz
            vis_map = vis_map_save{t,rep};
            set(h0.viz,'cdata',vis_map.map.');
            
            delete(h0.map);
            h0.map = drawEnv(y_true(~vis_map.map(y_true_linidx),:),1);
            
            h0.y = draw_traj_nx([],permute(tg_true(:,:,1:t,rep),[3 1 2 4]),'b:');

            for r = 1:num_robot
                h0.rob(r) = draw_pose_nx(h0.rob(r),permute(x_true(t,r,:,rep),[3 2 1]),'g',2.2);
                h0.fov(r) = draw_fov_nx(h0.fov(r),permute(x_true(t,r,:,rep),[3 2 1]),R(r).fov,R(r).r_sense);
            end
            %h0.ye = drawEnv([y_save{t,rep}(1:3:end,:)],1);
            delete(h0.ye);
            h0.ye = drawEnv([y_save{t,rep};y_exp_save{t,rep}],1);
            
            
            % Draw uncertainties
            h0.xcov = draw_covariances_nx(h0.xcov,permute(x_save(t,:,1:2,rep),[3 2 1]),...
                permute(x_cov_save(t,:,1:2,1:2,rep),[3 4 2 1 5]));
            tmp = y_save{t,rep};
            h0.ycov = draw_covariances_nx(h0.ycov,tmp(:,1:2).',y_cov_save{t,rep},'b');
            tmp = tg_save{t, rep};
            if ~isempty(tmp)
                h0.tg_cov = draw_covariances_nx(h0.tg_cov, tmp(:,1:2).', tg_cov_save{t,rep},'m');
            end
            title(sprintf('Time Step: %d',t));
            
            drawnow;
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
drawnow;
fnt_sz = 14;
%% Plot Covariance
repToShow = 1;
total_cost = zeros(run_len,num_rep);
total_x_cost = zeros(run_len,num_rep);
total_y_cost = zeros(run_len,num_rep);
for t = 1:run_len   
    for rep = 1:num_rep
        jC = xy_cholimat_save{t,rep};
        idx_x = 1:(num_robot*3);
        
        total_cost(t,rep) = gaussian_entropy_imatchol_nx(jC);
        total_y_cost(t,rep) = gaussian_entropy_imatchol_nx(jC((num_robot*3+1):end,(num_robot*3+1):end));
        
        jC_rearranged = jC( [(num_robot*3+1):end, idx_x], [(num_robot*3+1):end, idx_x]);
        jC_rearranged = qr(jC_rearranged);
        
        total_x_cost(t,rep) = gaussian_entropy_imatchol_nx(jC((end-num_robot*3+1):end,(end-num_robot*3+1):end));
    end
end

%%
figure('Color',[1 1 1],'Position',[200 200 500 200]);
plot(1:run_len,mean(total_cost,2),'b-','linewidth',2);
ylabel({'Joint Robot-Landmark','Pose Entropy [nats]'},'FontSize',fnt_sz);
xlabel('Time Steps','FontSize',fnt_sz);
set(gca,'fontsize',fnt_sz);
xlim([0,run_len]);
ylim([-250,50]);
set(gca,'YTick',[-250 -125 0 50]);
% figure('Color',[1 1 1],'Position',[200 200 500 200]);
% plot(total_y_cost,'b-');
% title('Total y cost');
% 
% figure('Color',[1 1 1],'Position',[100,277,1200,800]);
% plot(total_x_cost,'b-');
% title('Total x cost');

%% 
figure('Color',[1 1 1],'Position',[200 200 500 200]);
plot(1:run_len,total_cost(:,repToShow),'b-','linewidth',2);
ylabel({'Joint Robot-Landmark','Pose Entropy [nats]'},'FontSize',fnt_sz);
xlabel('Time Steps','FontSize',fnt_sz);
set(gca,'fontsize',fnt_sz);
xlim([0,run_len]);
ylim([-250,50]);
set(gca,'YTick',[-250 -125 0 50]);

%% Plot Robot Individual Covariances
repToShow = 1;
x_cost = zeros(run_len,num_robot,num_rep);
for rep = 1:num_rep
    for t = 1:run_len
        for k = 1:num_robot
            x_cost(t,k) = gaussian_entropy_nx(permute(x_cov_save(t,k,:,:,rep),[3 4 2 1 5]));
        end
    end
end

%%
figure('Color',[1 1 1],'Position',[200 200 500 200]);
plot(1:run_len,x_cost(:,:,repToShow));
xlabel('Time Steps','FontSize',fnt_sz);
ylabel({'Robot Pose','Entropy [nats]'},'FontSize',fnt_sz);
set(gca,'fontsize',fnt_sz);
xlim([0,run_len]);
ylim([-5,7]);

figure('Color',[1 1 1],'Position',[200 200 500 200]);
plot(1:run_len,mean(x_cost,3));
xlabel('Time Steps','FontSize',fnt_sz);
ylabel({'Robot Pose','Entropy [nats]'},'FontSize',fnt_sz);
set(gca,'fontsize',fnt_sz);
xlim([0,run_len]);
