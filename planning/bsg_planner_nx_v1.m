classdef bsg_planner_nx_v1 < handle
    properties (SetAccess = public, GetAccess = public)
        actions;        % discrete action set
        n_actions;      % number of actions
        action_indices; 
        robot_idx;
        num_robot;
        next_action_index;
        
        % BSG params
        J; % number of experts
        e;
        beta;
        eta;
        gamma;
        expert_weight;
        action_weight;
        action_prob_dist;
        loss;
        loss_estm;
        
        %
        som;
        tmm;
        smm;
        
    end
    methods
        function this = bsg_planner_nx_v1(num_robot, robot_idx, actions, n_time_step, samp, r_sense, fov, stdev_z)
            this.num_robot = num_robot;
            this.robot_idx = robot_idx;
            this.actions = actions;
            this.n_actions = length(actions);
            this.action_indices = 1:this.n_actions;
            this.next_action_index = ones(n_time_step, 1);
            learning_const = 8;
            this.J = ceil(log(n_time_step));
            this.e = learning_const*sqrt(log(this.J) / 2 / n_time_step);
            this.beta = 1 / (n_time_step - 1);
            this.eta = zeros(this.J, 1);
            for j = 1 : this.J
                this.eta(j) = learning_const*sqrt(log(this.n_actions*n_time_step) / pow2(j-1) / this.n_actions);
            end
            this.gamma = this.eta / 2;
            this.expert_weight = 1 / this.J * ones(n_time_step, this.J);
            this.action_weight = 1 / this.n_actions * ones(n_time_step, this.J, this.n_actions);
            this.action_prob_dist = zeros(n_time_step, this.n_actions);
            this.loss = zeros(n_time_step, this.n_actions);
            this.loss_estm = zeros(n_time_step, this.n_actions);
                        
            this.som.r_sense = r_sense;
            this.som.fov = fov;
            this.som.stdev_z = stdev_z;
            
            this.smm.samp = samp;
            this.smm.f = @(x,u) dd_motion_model(x(1:3,:),u,this.smm.samp,true);
            %this.smm.f = @(x,u) point_mass_motion_model(x(1:3,:),u,this.smm.samp);
            
        end
        
        function update_experts(this, t)
            % update weights
            for j = 1:this.J
                %nxt_a_idx = this.next_action_index(t);
                this.loss_estm(t, this.next_action_index(t)) = this.loss(t, this.next_action_index(t)) /...
                    (this.action_prob_dist(t, this.next_action_index(t)) + this.gamma(j));
                v = zeros(this.n_actions,1);
                for i = 1 : this.n_actions
                    v(i) = this.action_weight(t, j, i) * exp(-this.eta(j)*this.loss_estm(t, i));                    
                        %v(i) = this.action_weight(t, j, i) * exp(this.eta(j)*(1-this.loss_estm(t, i)));
                end
                W_t = sum(v);
                this.action_weight(t+1,j,:) = this.beta*W_t/this.n_actions + (1-this.beta)*v;
                    
                this.expert_weight(t+1,j) = this.expert_weight(t, j)*...
                    exp(-this.e*this.loss_estm(t,:)*(reshape(this.action_weight(t, j,:), this.n_actions,[])/norm(squeeze(this.action_weight(t, j,:)),1)));
                
                %this.expert_weight(t+1,j) = this.expert_weight(t, j)*...
                %   exp(this.e*(1-this.loss_estm(t,:))*(reshape(this.action_weight(t, j,:), this.n_actions,[])/norm(squeeze(this.action_weight(t, j,:)),1)));
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
                
        function update_action_prob_dist(this, t)
%             if t == 1
%                 this.action_prob_dist(t, :) = 1/this.n_actions*ones(1, this.n_actions);
%             end
            q_t = this.expert_weight(t,:);    % 1 * J
            p_t = this.action_weight(t,:,:);  % J * n_actions
            
            this.action_prob_dist(t, :) = squeeze(q_t) * squeeze(p_t);  % 1 * n_actions
            if sum(isnan(this.action_prob_dist(t, :))) > 0
                warning("nan value is not valid");
            end
            if abs(sum(this.action_prob_dist(t, :)) - 1) > 1e-4
                error("incorrect prob distribution");
            end
        end
        
        function [loss, obj_mat] = losses_after_action(this, t, robot_idx, x,  y, u, prev_loss, prev_obj_mat)
            % Description: 
            % given fixed actions of robots whose indexes are smaller than 
            % robot_idx and observed targets at current time t+1, calculate 
            % objective function after robot(robot_idx) having executed
            % specified action.
            % Input:
            % t: time t
            % x: robot positions
            % y: target positions
            % Output:
            % obj_mat: obj_mat(kk) contains kkth target of corresponding 
            % sum -1/(r_i.^2) with r being the distance btw kkth target and
            % ith robot that detects kkth target.
            % 
            % loss: 1 
            
            % f(a_i | A_{i-1})
            cur_x = this.smm.f(x, u);
            obj_mat = prev_obj_mat;
            num_tg = size(y, 1);
            for kk = 1 : num_tg
                cur_y = y(kk, 1:2)';  
                valid = visibility(cur_x, cur_y, this.som.r_sense, this.som.fov);
                if valid
                    range = norm(cur_x(1:2) - cur_y);
                    obj_mat(kk) = obj_mat(kk) - 1 / (range.^2);
                end
            end

            cur_reward = reward_function(obj_mat);
            %prev_reward = 1 - prev_loss;
            prev_reward = reward_function(prev_obj_mat);
            dimi_reward = cur_reward - prev_reward;
            % sanity check
            if dimi_reward < 0 || dimi_reward > 1
                error('marginal gain can not be negative or larger than 1');
            end
            loss = 1 - dimi_reward;
            this.loss(t, this.next_action_index(t)) = loss;
        end
                
        % helper functions
%         function [min_y, min_y_cov, max_y, max_y_cov] = get_minmax_y(y,y_cov)
%             %   y = 2 x num_y = target states
%             %   y_cov = 2 x 2 x num_y = target covariances
%             
%             min_y = []; min_y_cov = inf;
%             max_y = []; max_y_cov = -inf;
%             
%             if( ~isempty(y) )
%                 num_y = size(y,2);
%                 for k = 1:num_y
%                     ld = logdet_nx(y_cov(:,:,k));
%                     if( ld < min_y_cov )
%                         min_y_cov = ld;
%                         min_id = k;
%                     end
%                     
%                     if( ld > max_y_cov )
%                         max_y_cov = ld;
%                         max_id = k;
%                     end
%                 end
%                 
%                 min_y = y(:,min_id); min_y_cov = y_cov(:,:,min_id);
%                 max_y = y(:,max_id); max_y_cov = y_cov(:,:,max_id);
%             end
%         end
%          function [att,att_cholimat] = get_attractor(this,x,x_cov,y,y_cov,exp_y_cl)
%             % INPUT:
%             %   x = 3 x 1 = sensor state
%             %   x_cov = 3 x 3 = sensor covariance matrix
%             %
%             %   y = 2 x num_y = target states
%             %   y_cov = 2 x 2 x num_y = target covariances
%             %
%             % OUTPUT:
%             %   att = num_a x 2
%             %   att_cholimat = 2*num_a x 2*num_a
%             %
%             
%             [min_y, ~, max_y, max_y_cov] = this.get_minmax_y(y,y_cov);
%             exp_y_cl = exp_y_cl(:);
%             
%             % determine the state
%             if( ~isempty(y) )
%                 dist_to_max_y = norm(max_y(1:2) - x(1:2));
%             end
%             if( ~isempty(exp_y_cl) )
%                 dist_to_exp_y = norm(exp_y_cl(1:2) - x(1:2));
%             end
%             
%             % Should we enter localize?
%             if( ~strcmpi(this.att_state,'localize') && logdet_nx(x_cov) > 1)
%                 this.att_state = 'localize';
%             end
%             % Should we exit localize?
%             if( strcmpi(this.att_state,'localize') && logdet_nx(x_cov) < -3.5)
%                 this.att_state = 'none';
%             end
%             
%             % Should we enter map?
%             if( ~strcmpi(this.att_state,'localize') && ~isempty(y) && logdet_nx(max_y_cov) > 6 ...
%                     && dist_to_max_y > 8)
%                 this.att_state = 'map';
%             end
%             % Should we exit map?
%             if( strcmpi(this.att_state, 'map') && ~isempty(y) && logdet_nx(max_y_cov) < 3 )
%                 this.att_state = 'none';
%             end
%             
%             % Should we enter explore
%             if( ~strcmpi(this.att_state,'localize') && ~strcmpi(this.att_state,'map')...
%                     && ~isempty(exp_y_cl) && dist_to_exp_y > 15 )
%                 this.att_state = 'explore';
%             end
%             % Should we exit explore
%             if( strcmpi(this.att_state,'explore') && ~isempty(exp_y_cl) && dist_to_exp_y < 10 )
%                 this.att_state = 'none';
%             end
%             
%             
%             % Choose the attractor
%             dev = 1:1:23;%[3,6,8,12,16,18,20];
%             switch lower(this.att_state)
%                 case 'localize'
%                     if( ~isempty(y) )
%                         dv = min_y(1:2) - x(1:2);
%                         dv = bsxfun(@times,dv(:)/norm(dv),dev);
%                         att = transpose(bsxfun(@plus,x(1:2),dv));
%                         att_cholimat = 0.0005 * speye(2*numel(dev));
%                         %att_cholimat([13,14,19,20,33,34],[13,14,19,20,33,34]) = 10;
%                         fprintf('LOC ATTRACTOR\n');
%                     else
%                         att = []; att_cholimat = [];
%                     end
%                     
%                 case 'map'
%                     if( ~isempty(y) )
%                         dv = max_y(1:2) - x(1:2);
%                         dv = bsxfun(@times,dv(:)/norm(dv),dev);
%                         att = transpose(bsxfun(@plus,x(1:2),dv));
%                         att_cholimat = 0.0005 * speye(2*numel(dev));
%                         %att_cholimat([13,14,19,20,33,34],[13,14,19,20,33,34]) = 10;
%                         fprintf('MAP ATTRACTOR\n');
%                     else
%                         att = []; att_cholimat = [];
%                     end
%                 case 'explore'
%                     if(~isempty(exp_y_cl))
%                         dv = exp_y_cl(1:2) - x(1:2);
%                         dv = bsxfun(@times,dv(:)/norm(dv),dev);
%                         att = transpose(bsxfun(@plus,x(1:2),dv));
%                         att_cholimat = 0.0005 * speye(2*numel(dev));
%                         %att_cholimat([13,14,19,20,33,34],[13,14,19,20,33,34]) = 10;
%                         fprintf('EXP ATTRACTOR\n');
%                     else
%                         att = []; att_cholimat = [];
%                     end
%                 otherwise
%                     att = [];
%                     att_cholimat = [];
%             end
%         end
    end
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


