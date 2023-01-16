classdef osg_planner_nx_v1 < handle
    properties (SetAccess = protected, GetAccess = public)
        actions;        % discrete action set
        n_actions;      % number of actions
        action_indices; 
        robot_idx;
        num_robot;
        ncov;
        att_state = 'none';
        att_save = [];
        % OSG params
        J; % number of experts
        g;
        beta;
        gamma;
        expert_weight;
        action_weight;
        action_prob_dist;
        loss;
        
        %
        som;
        tmm;
        smm;
        
    end
    methods
        function this = osg_planner_nx_v1(num_robot, robot_idx, actions, n_time_step, samp,r_sense, fov, stdev_z)
            this.num_robot = num_robot;
            this.robot_idx = robot_idx;
            this.actions = actions;
            this.n_actions = length(actions);
            this.action_indices = 1:this.n_actions;
            
            this.J = ceil(log(n_time_step));
            this.g = sqrt(log(this.J) / n_time_step);
            this.beta = 1 / n_time_step;
            this.gamma = zeros(this.J, 1);
            for j = 1 : this.J
                this.gamma(j) = sqrt(log(this.n_actions*n_time_step)/pow2(j-1));
            end
            this.expert_weight = 1 / this.J * ones(n_time_step, this.J);
            this.action_weight = 1 / this.n_actions * ones(n_time_step, this.J, this.n_actions);
            this.action_prob_dist = zeros(n_time_step, this.n_actions);
            this.loss = zeros(n_time_step, this.n_actions);
            this.ncov = zeros(n_time_step, this.n_actions);
            
            this.som.r_sense = r_sense;
            this.som.fov = fov;
            this.som.stdev_z = stdev_z;
                        
            this.tmm.A = @(x,u) dd_motion_model_jacobian_x(x,u,samp,true); % (3 x 3 x num_u x num_x)
            this.tmm.W = 1/40*speye(3);%diag([0.035; 0.035; 0.014]); %diag([stdev_u(1); stdev_u(1); stdev_u(2)].^2);
            this.tmm.chol_inv_W = chol(sparse(this.tmm.W)\speye(size(this.tmm.W)));
            
            this.smm.samp = samp;
            this.smm.f = @(x,u) dd_motion_model(x(1:3,:),u,this.smm.samp,true);
            
        end
        
        function update_experts(this, t)
            % update weights
            for j = 1:this.J
                v = zeros(this.n_actions,1);
                for i = 1 : this.n_actions
                    v(i) = this.action_weight(t, j, i) * exp(-this.gamma(j)*this.loss(t, i));
                end
                W_t = sum(v);
                this.action_weight(t+1,j,:) = this.beta*W_t + (1-this.beta)*v;
                    
                this.expert_weight(t+1,j) = this.expert_weight(t, j)*...
                    exp(-this.g*this.loss(t,:)*(reshape(this.action_weight(t, j,:), this.n_actions,[])/norm(squeeze(this.action_weight(t, j,:)),1)));
            end
            % normalize weights by 1-Norm
            for j = 1:this.J
                this.action_weight(t+1, j, :) = this.action_weight(t+1, j, :) / norm(squeeze(this.action_weight(t+1, j, :)), 1);
                if sum(isnan(this.action_weight(t+1, j, :))) > 0
                    warning("Nan value is not valid.");
                end
            end
            this.expert_weight(t+1,:) = this.expert_weight(t+1,:) / norm(squeeze(this.expert_weight(t+1,:)), 1);
            
        end
        

        
        function get_action_prob_dist(this, t)
%             if t == 1
%                 this.action_prob_dist(t, :) = 1/this.n_actions*ones(1, this.n_actions);
%             end
            q_t = this.expert_weight(t,:);    % 1 * J
            p_t = this.action_weight(t,:,:);  % J * n_actions
            
            this.action_prob_dist(t, :) = squeeze(q_t) * squeeze(p_t);  % 1 * n_actions
            if sum(isnan(this.action_prob_dist(t, :))) > 0
                warning("nan value is not valid");
            end
        end
        
        function losses_after_action(this, t,robot_idx, x, x_cov,  y, xy_cholimat, exp_y, fixed_x, fixed_u)
            %% still undefined
            % after take action for robot(robot_idx), the loss function
            % becomes ...
            % TODO: 
            % 1. get R_prev
            % 2. for every u, get A, W, V, H
            % 3. using SRIF to update R_Prev and calculate cost.
            % get A,W,V,H to update,  
            % @INPUT: 
            % robot_idx: current robot id, for robots that idx
            %            is 1:robot_idx-1,  loss has been updated.
            %        x:  3 x 1
            %        y: num_y x 2
            %xy_cholimat: cholimat of robots' poses of last time step and current targets' positions.     
            %[att,att_cholimat] = this.get_attractor(x,x_cov,y,y_cov,exp_y_cl);
            
            if isempty(y)
                numel_y = 0;
                y_init = exp_y;
            else
                y = y(:,1:2);
                numel_y = numel(y);
                y_init = [y; exp_y];
            end
            obj_fun = @logdetchol_wz;
%             num_all_x = size(xy_cholimat, 1) - numel_y;
            num_all_x = this.num_robot*3;

            %this.som.H = @(x) H_rb_xy(transpose(x(1:3,:)),y_init,this.som.r_sense,this.som.fov,num_all_x-3);
            this.som.H = @(rx, num_sep) H_rb_xy(transpose(rx(1:3,:)),y_init,this.som.r_sense,this.som.fov, num_sep);
            numel_y_init = numel(y_init);
            numel_exp_y = numel(exp_y);
            idx = 1:numel_y_init*robot_idx;
            this.som.V = sparse(idx,idx,repmat(this.som.stdev_z.^2,(numel_y_init/2)*robot_idx,1));
            this.som.chol_inv_V = sparse(idx,idx,repmat(1./this.som.stdev_z,(numel_y_init/2)*robot_idx,1));
            
            x = x(:);
            nX = this.smm.f(x, this.actions);
            nX = reshape(nX, 3, []);
            num_nx = size(nX, 2);
            ncov = zeros(1,num_nx);
            if size(xy_cholimat, 1) ~= num_all_x + numel_y
                xy_cholimat = blkdiag(xy_cholimat, zeros(abs(num_all_x + numel_y - size(xy_cholimat, 1))));
                disp("change");

            end
            R_t = [xy_cholimat, sparse(num_all_x+numel_y, numel_exp_y);
                   sparse(numel_exp_y, num_all_x+numel_y),speye(numel_exp_y)];
            %% 
            fixed_x = reshape(cell2mat(fixed_x),3,[]);
            fixed_u = reshape(cell2mat(fixed_u),2,[]);
            if robot_idx > 1
                
                % fixed_H = this.som.H(fixed_x(:, 1:robot_idx-1), num_all_x-(robot_idx-1)*3);
                % H_temp = zeros(numel_y_init, size(R_t,1), robot_idx-1);
                fixed_H = zeros(numel_y_init*(robot_idx-1), size(R_t,1));
                fixed_A = zeros(3*(robot_idx-1), 3*(robot_idx-1));
                for i = 1:robot_idx-1
                    fixed_H(numel_y_init*(i-1)+1:numel_y_init*i,3*(i-1)+1:end) = this.som.H(fixed_x(:, i), num_all_x-i*3);                    
                    
                    fixed_A(3*(i-1)+1:3*(i-1)+3,3*(i-1)+1:3*(i-1)+3) = dd_motion_model_jacobian_x(fixed_x(:, i), fixed_u(:, i), this.smm.samp);
                    
                end
                %TODO: use fixed_A and fixed_H to compute f(A_{t-1})
                idx = 1:numel_y_init*(robot_idx-1);
                fixed_chol_inv_V = sparse(idx,idx,repmat(1./this.som.stdev_z,(numel_y_init/2)*(robot_idx-1),1));
                idx = 1:3*(robot_idx-1);
                fixed_ciW = sparse(idx,idx,repmat(diag(this.tmm.chol_inv_W),robot_idx-1,1));
                fixed_S = srif_pe(R_t, fixed_A, fixed_ciW, fixed_H, fixed_chol_inv_V);  
                %old_cost = -sum(abs(diag(fixed_S)));
                old_cost = -obj_fun(fixed_S);
%               old_cost = -logdetchol_nx(fixed_S);
                %old_cost = -weighted_logdetchol_nx1(fixed_S, this.num_robot*3);
            end
            
            A = this.tmm.A(x, this.actions);

            idx = 1:3*robot_idx;
            ciW = sparse(idx,idx,repmat(diag(this.tmm.chol_inv_W),robot_idx,1));
            
            for i = 1:num_nx
                %TO DO:
                %[A, W, H, V] =
                % srif_pe
                % get cost
                H_cur = this.som.H(nX(:, i),num_all_x-robot_idx*3);
                H_cur = [zeros(numel_y_init, size(R_t,1) - size(H_cur,2)) this.som.H(nX(:, i),num_all_x-robot_idx*3)];
                
                if robot_idx > 1
                    % patch H of different sizes. 
                    A_cur = blkdiag(fixed_A, A(:,:,i));
                    H_cur = [fixed_H; H_cur];
                    S = srif_pe(R_t, A_cur, ciW, H_cur, this.som.chol_inv_V);
                    ncov(i) =  - obj_fun(S) - old_cost;
%                     ncov(i) = old_cost-logdetchol_nx(S);
                    %ncov(i) = old_cost-weighted_logdetchol_nx1(S, this.num_robot*3);
                    if ncov(i) == Inf || ncov(i) == -Inf
                        warning("unwanted value");
                    end
                    
                else
                    A_cur = A(:,:,i);
                    H_cur = [zeros(numel_y_init, size(R_t,1) - size(H_cur,2)) H_cur];
                    S = srif_pe(R_t, A_cur, ciW, H_cur, this.som.chol_inv_V);
                    ncov(i) = -obj_fun(S);
%                     ncov(i) = -logdetchol_nx(S);
                    %ncov(i) = -weighted_logdetchol_nx1(S, this.num_robot*3);
                    if ncov(i) == Inf || ncov(i) == -Inf || ncov(i) > 0
                        warning("unwanted value");
                    end
                end                            
            end
            this.ncov(t, :) = ncov - max(ncov);
            
            this.loss(t, :) = this.ncov(t, :)/max(abs(this.ncov(t, :)));    % in [-1, 0], -1 with biggest weight.
             g=sprintf('%2f ', this.loss(t,:));
             sprintf('loss vector of robot %i is %s \n', robot_idx, g)
            if sum(isnan(this.loss(t, :))) > 0
                warning("nan value is not valid");
            end
        end
        
        % helper functions
        function [min_y, min_y_cov, max_y, max_y_cov] = get_minmax_y(y,y_cov)
            %   y = 2 x num_y = target states
            %   y_cov = 2 x 2 x num_y = target covariances
            
            min_y = []; min_y_cov = inf;
            max_y = []; max_y_cov = -inf;
            
            if( ~isempty(y) )
                num_y = size(y,2);
                for k = 1:num_y
                    ld = logdet_nx(y_cov(:,:,k));
                    if( ld < min_y_cov )
                        min_y_cov = ld;
                        min_id = k;
                    end
                    
                    if( ld > max_y_cov )
                        max_y_cov = ld;
                        max_id = k;
                    end
                end
                
                min_y = y(:,min_id); min_y_cov = y_cov(:,:,min_id);
                max_y = y(:,max_id); max_y_cov = y_cov(:,:,max_id);
            end
        end
        function [att,att_cholimat] = get_attractor(this,x,x_cov,y,y_cov,exp_y_cl)
            % INPUT:
            %   x = 3 x 1 = sensor state
            %   x_cov = 3 x 3 = sensor covariance matrix
            %
            %   y = 2 x num_y = target states
            %   y_cov = 2 x 2 x num_y = target covariances
            %
            % OUTPUT:
            %   att = num_a x 2
            %   att_cholimat = 2*num_a x 2*num_a
            %
            
            [min_y, ~, max_y, max_y_cov] = this.get_minmax_y(y,y_cov);
            exp_y_cl = exp_y_cl(:);
            
            % determine the state
            if( ~isempty(y) )
                dist_to_max_y = norm(max_y(1:2) - x(1:2));
            end
            if( ~isempty(exp_y_cl) )
                dist_to_exp_y = norm(exp_y_cl(1:2) - x(1:2));
            end
            
            % Should we enter localize?
            if( ~strcmpi(this.att_state,'localize') && logdet_nx(x_cov) > 1)
                this.att_state = 'localize';
            end
            % Should we exit localize?
            if( strcmpi(this.att_state,'localize') && logdet_nx(x_cov) < -3.5)
                this.att_state = 'none';
            end
            
            % Should we enter map?
            if( ~strcmpi(this.att_state,'localize') && ~isempty(y) && logdet_nx(max_y_cov) > 6 ...
                    && dist_to_max_y > 8)
                this.att_state = 'map';
            end
            % Should we exit map?
            if( strcmpi(this.att_state, 'map') && ~isempty(y) && logdet_nx(max_y_cov) < 3 )
                this.att_state = 'none';
            end
            
            % Should we enter explore
            if( ~strcmpi(this.att_state,'localize') && ~strcmpi(this.att_state,'map')...
                    && ~isempty(exp_y_cl) && dist_to_exp_y > 15 )
                this.att_state = 'explore';
            end
            % Should we exit explore
            if( strcmpi(this.att_state,'explore') && ~isempty(exp_y_cl) && dist_to_exp_y < 10 )
                this.att_state = 'none';
            end
            
            
            % Choose the attractor
            dev = 1:1:23;%[3,6,8,12,16,18,20];
            switch lower(this.att_state)
                case 'localize'
                    if( ~isempty(y) )
                        dv = min_y(1:2) - x(1:2);
                        dv = bsxfun(@times,dv(:)/norm(dv),dev);
                        att = transpose(bsxfun(@plus,x(1:2),dv));
                        att_cholimat = 0.0005 * speye(2*numel(dev));
                        %att_cholimat([13,14,19,20,33,34],[13,14,19,20,33,34]) = 10;
                        fprintf('LOC ATTRACTOR\n');
                    else
                        att = []; att_cholimat = [];
                    end
                    
                case 'map'
                    if( ~isempty(y) )
                        dv = max_y(1:2) - x(1:2);
                        dv = bsxfun(@times,dv(:)/norm(dv),dev);
                        att = transpose(bsxfun(@plus,x(1:2),dv));
                        att_cholimat = 0.0005 * speye(2*numel(dev));
                        %att_cholimat([13,14,19,20,33,34],[13,14,19,20,33,34]) = 10;
                        fprintf('MAP ATTRACTOR\n');
                    else
                        att = []; att_cholimat = [];
                    end
                case 'explore'
                    if(~isempty(exp_y_cl))
                        dv = exp_y_cl(1:2) - x(1:2);
                        dv = bsxfun(@times,dv(:)/norm(dv),dev);
                        att = transpose(bsxfun(@plus,x(1:2),dv));
                        att_cholimat = 0.0005 * speye(2*numel(dev));
                        %att_cholimat([13,14,19,20,33,34],[13,14,19,20,33,34]) = 10;
                        fprintf('EXP ATTRACTOR\n');
                    else
                        att = []; att_cholimat = [];
                    end
                otherwise
                    att = [];
                    att_cholimat = [];
            end
        end
    end

      %         function get_losses(this, t, rep, y_d, x, robot_idx)
%             % Description:
%             % get losses by measuring the distance between robots and
%             % targets.
%             % INPUT:
%             % t: time idx
%             % y_d: position of dynamic target,
%             % 3*num_y_d*(run_len+1)*num_rep
%             % x: position of robots
%             % (run_len+1)*num_robot*3*num_rep)
%             % robot_idx: index of robot
%             losses = zeros(1, this.n_actions);
%             obj_action = zeros(1, this.n_actions);
%             time_sample = 0.5;
%             for i = 1:this.n_actions
%                 action = this.action_indices(i);
%                 % if apply actions(i) to x(t-1), current state would be
%                 if t > 1
%                     cur_pos_x = dd_motion_model(squeeze(x(t-1, robot_idx, :, rep)), this.actions(:, action), time_sample);
%                 else
%                     cur_pos_x = dd_motion_model(squeeze(x(1, robot_idx, :, rep)), this.actions(:, action), time_sample);
%                 end
%                 num_target = size(y_d, 2);
%                 for j = 1:num_target
%                     obj_action(i) = obj_action(i) + 1 / (norm(y_d(1:2, j, t,rep) - cur_pos_x(1:2) + 0.0001));
%                 end
%             end
%             obj_action = normalize(obj_action,'range');
%             
%             % 
%             if robot_idx == 1
%                 losses = - obj_action / max(obj_action);
%                 
%             else
%                 curr_obj = 0;
%                 for j = 1 : size(y_d, 2)
%                     min_dist = inf;
%                     for r = 1 : robot_idx-1
%                         if norm(y_d(1:2, j, t,rep) - squeeze(x(t, r, 1:2, rep)) + 0.001) < min_dist
%                             min_dist = norm(y_d(1:2, j, t,rep) - squeeze(x(t, r, 1:2, rep)) + 0.0001);
%                         end
%                     end
%                     curr_obj = curr_obj + 1 / min_dist;
%                 end
%                 
%                 for i = 1:this.n_actions
%                     temp_obj = 0;
%                     action = this.action_indices(i);
%                     if t > 1
%                         cur_pos_x = dd_motion_model(squeeze(x(t-1, robot_idx, :, rep)), this.actions(:, action), time_sample);
%                     else
%                         cur_pos_x = dd_motion_model(squeeze(x(1, robot_idx, :, rep)), this.actions(:, action), time_sample);
%                     end
%                     
%                     for j = 1 : size(y_d, 2)
%                         min_dist = norm(y_d(1:2, j, t,rep) - cur_pos_x(1:2) + 0.001);
%                         for r = 1 : robot_idx-1
%                             if norm(y_d(1:2, j, t,rep) - squeeze(x(t, r, 1:2, rep)) + 0.001) < min_dist
%                                 min_dist = norm(y_d(1:2, j, t,rep) - squeeze(x(t, r, 1:2, rep)) + 0.0001);
%                             end
%                         end
%                         temp_obj = temp_obj + 1 / min_dist;
%                     end
%                     losses(i) = (curr_obj - temp_obj) / max(obj_action);
%                 end
%                 
%             end
%             this.loss(t, :) = losses;            
%         end  
end
function d = weighted_trace(R, dy)
    scale = 1;
    
end
function d = weighted_logdetchol_nx1(R,dy)
scale = 1; % relative importance of localization 1 = equal
            % 1 = equal to mapping
            % >1 = more important than mapping
            % ld_Us: for target
            % ld_Uy: for robots
ld_Us = 2*sum(log(abs(diag(R((dy+1):end,(dy+1):end)))));
R = R([dy+1:end,1:dy],[dy+1:end,1:dy]);
R = qr(R);
ds = (size(R,1) - dy);
ld_Uy = 2*sum(log(abs(diag(R((ds+1):end,(ds+1):end)))));

d = ld_Us/ds + scale*ld_Uy/dy;
%d0 = 2*sum(log(abs(diag(R))));  % logdet(R.'*R)
%d1 = 2*sum(log(abs(diag(R((dy+1):end,(dy+1):end)))));
%ds = scale*(size(R,1) - dy);   % static dimension
%d = ds*d0 - (ds-dy)*d1;
end


