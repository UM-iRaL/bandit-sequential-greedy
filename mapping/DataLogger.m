classdef DataLogger < handle
    properties (SetAccess = public, GetAccess = public)
        x_true = [];
        y_true = [];
        u_true = [];
        z_true = {};
        u_plan_true = {};
        
        x_estm = [];
        x_cov = [];
        y_estm = [];
        y_cov = [];
        
        % data association
        landmark_seen = [];
        %landmark_map = {};
        %reverse_landmark_map = [];
        
        % Exploration and visibility landmarks
        y_exp_save = {};
        %y_att_save = {};
        vis_map = {};
        exp_map = {};
        exp_map_init;
        
        % for plotting
        hndl = [];
    end
    
    methods
        function this = DataLogger(prm)
            % Takes parameters from simulation_parameters()
            % and sets up the logging environment and
            % initializes the simulation
            
            % x_true
            this.x_true = zeros(prm.dim_x,prm.num_x,prm.run_len+1,prm.num_rep);
            this.x_true(:,1,1,:) = repmat( [-31.7+5;-2;0], 1,prm.num_rep);
            this.x_true(:,2,1,:) = repmat( [-30+5;-6;degtorad(90)], 1,prm.num_rep);
            this.x_true(:,3,1,:) = repmat( [-33+5;4;degtorad(0)],1,prm.num_rep);
            %this.x_true(:,1,1,:) = repmat([-20;-20;0],1,prm.num_rep);
            %this.x_true(:,2,1,:) = repmat([50;50;deg2rad(90)],1,prm.num_rep);
            %this.x_true(:,3,1,:) = repmat([15;14;deg2rad(0)],1,prm.num_rep);
            
            % y_true
            this.y_true = zeros(prm.dim_y,prm.num_y,prm.run_len+1,prm.num_rep);
            %this.y_true(:,1,1,:) = repmat([10;10],1,prm.num_rep);
            %this.y_true(:,2,1,:) = repmat([11;11],1,prm.num_rep);
            %this.y_true(:,3,1,:) = repmat([-5;5],1,prm.num_rep);
            %for yy = 4:prm.num_y
            %    this.y_true(:,yy,1,:) = repmat(80*rand(2,1)-40,1,prm.num_rep);
            %end

            % Sample from free space
            %tmp_min = [-40;-30]; tmp_max = [40;30];
            tmp_min = [-29;-18]; tmp_max = [29;18];
            if( isfield(prm,'MAP') )
                ptr = 0;
                while( ptr < prm.num_y )
                    y = bsxfun(@plus,tmp_min, bsxfun(@times,(tmp_max-tmp_min),rand(2,10*prm.num_y))); 
                    %y = bsxfun(@plus,prm.MAP.min, bsxfun(@times,(prm.MAP.max-prm.MAP.min),rand(2,10*prm.num_y)));     %2 x num_y
                    
                    valid = check_collision(y, prm.MAP);
                    y(:,~valid) = [];
                    num_valid = sum(valid);
                    num_left = prm.num_y - ptr;
                    num_valid = min(num_valid,num_left);
                    this.y_true(:,ptr+1:ptr+num_valid,1,:) = repmat( y(:,1:num_valid), 1,1,prm.num_rep );
                    ptr = ptr+num_valid;
                end
            else
                y = bsxfun(@plus,tmp_min, bsxfun(@times,(tmp_max-tmp_min),rand(2,prm.num_y)));
                this.y_true(:,:,1,:) = repmat( y, 1,1,prm.num_rep );
            end

            % u_true
            this.u_true = zeros(prm.dim_u,prm.num_x,prm.run_len,prm.num_rep);
            this.z_true = cell(prm.num_x,prm.run_len,prm.num_rep);
            this.u_plan_true = cell(prm.num_x,prm.run_len,prm.num_rep);

            % estimates
            this.y_estm = nan(prm.dim_y,prm.num_y,prm.run_len+1,prm.num_rep);
            this.y_cov = nan(prm.dim_y,prm.dim_y,prm.num_y,prm.run_len+1,prm.num_rep);
            
            %{
            for yy = 1:prm.num_y
                this.y_cov(:,:,yy,1,:) = repmat(eye(prm.dim_y),1,1,prm.num_rep);
            end
            
            for rep = 1:prm.num_rep
                for yy = 1:prm.num_y
            
                    this.y_estm(:,yy,1,rep)= this.y_true(:,yy,1,rep) + chol(this.y_cov(:,:,yy,1,:)).'*randn(prm.dim_y,1);
            
                    this.y_estm(1,yy,1,rep) = max(this.y_estm(1,yy,1,rep),tmp_min(1));
                    this.y_estm(1,yy,1,rep) = min(this.y_estm(1,yy,1,rep),tmp_max(1));
                    this.y_estm(2,yy,1,rep) = max(this.y_estm(2,yy,1,rep),tmp_min(2));
                    this.y_estm(2,yy,1,rep) = min(this.y_estm(2,yy,1,rep),tmp_max(2));
            
                end
            end
            
            for yy = 1:prm.num_y
                this.y_cov(:,:,yy,1,:) = repmat(eye(prm.dim_y),1,1,prm.num_rep);
            end
            %}
            
            % Data association
            this.landmark_seen = false(prm.num_y,prm.num_rep);
            %for rep = 1:prm.num_rep
            %    this.landmark_map{rep} = containers.Map('KeyType','double','ValueType','double');
            %end
            %this.reverse_landmark_map = zeros(prm.num_y,prm.num_rep);
            
            % exploration and visibility landmarks
            this.y_exp_save = cell(prm.run_len,prm.num_rep);
            %this.y_att_save = cell(prm.run_len,prm.num_rep);
            this.exp_map = cell(prm.run_len+1,prm.num_rep);
            this.vis_map = cell(prm.run_len+1,prm.num_rep);

            for rep = 1:prm.num_rep
                this.exp_map{1,rep} = init_map_nx(tmp_min,tmp_max,3,'logical');
                %this.exp_map{1,rep} = init_map_nx(prm.MAP.min,prm.MAP.max,3,'logical');
                if( isfield(prm,'MAP') )
                    this.vis_map{1,rep} = prm.MAP;
                else
                    this.vis_map{1,rep} = init_map_nx(tmp_min,tmp_max,0.025,'logical');
                end
                this.vis_map{1,rep}.map = true(size(this.vis_map{1,rep}.map));
            end

            this.exp_map_init = init_map_nx(tmp_min,tmp_max,3,'logical');
            %this.exp_map_init = init_map_nx(prm.MAP.min,prm.MAP.max,3,'logical');
            [this.exp_map_init.xmap, this.exp_map_init.ymap] = ...
                ndgrid(this.exp_map_init.pos{1},this.exp_map_init.pos{2});            
        end
        
        function exp_y = update_vis_exp(this,x,prm,rep,t)
            % x = num_x x 3 = robot states
            pt = max(1,t-1);
            
            % Remove visible exploration landmarks
            if( isfield(prm,'MAP') )
                this.exp_map{t,rep} = DataLogger.remove_visible_exp(this.exp_map{pt,rep},...
                    this.exp_map_init.xmap, this.exp_map_init.ymap,...
                    x,prm.r_sense,prm.fov,prm.MAP);
                
                % Remove visibility
                this.vis_map{t,rep} = DataLogger.remove_visible_vis(this.vis_map{pt,rep},x,prm.r_sense,prm.fov,prm.MAP);

                % Determine Frontier exploration landmarks
                this.exp_map{t,rep} = DataLogger.add_frontier(this.exp_map{t,rep},this.vis_map{t,rep},x,prm.r_sense,prm.fov,prm.MAP);                
            else
                this.exp_map{t,rep} = DataLogger.remove_visible_exp(this.exp_map{pt,rep},...
                    this.exp_map_init.xmap, this.exp_map_init.ymap,...
                    x,prm.r_sense,prm.fov);
                % Remove visibility
                this.vis_map{t,rep} = DataLogger.remove_visible_vis(this.vis_map{pt,rep},x,prm.r_sense,prm.fov);

                % Determine Frontier exploration landmarks
                this.exp_map{t,rep} = DataLogger.add_frontier(this.exp_map{t,rep},this.vis_map{t,rep},x,prm.r_sense,prm.fov);                
            end
            
            exp_y = [this.exp_map_init.xmap(this.exp_map{t,rep}.map), ...
                this.exp_map_init.ymap(this.exp_map{t,rep}.map)];
            this.y_exp_save{t,rep} = [exp_y,ones(size(exp_y,1),1)];
                
        end
        
        function init_viz(this,prm,rep,t)
            
            if( nargin < 4 )
                t = 1;
            end
            
            this.hndl.f = figure('Color',[1 1 1],'Position',[100,277,1200,800]);
            if(isfield(prm,'MAP'))
                this.hndl.map = imagesc([this.vis_map{t,rep}.pos{1}(1);this.vis_map{t,rep}.pos{1}(end)],...
                    [this.vis_map{t,rep}.pos{2}(1);this.vis_map{t,rep}.pos{2}(end)],...
                    (this.vis_map{t,rep}.map|prm.MAP.map).');
            else
                this.hndl.map = imagesc([this.vis_map{t,rep}.pos{1}(1);this.vis_map{t,rep}.pos{1}(end)],...
                    [this.vis_map{t,rep}.pos{2}(1);this.vis_map{t,rep}.pos{2}(end)],(this.vis_map{t,rep}.map).');                
            end
            cbone = bone; colormap(cbone(end:-1:(end-30),:));
            
            hold on;
            this.hndl.xp = draw_pose_nx([],this.x_true(:,:,t,rep),'r',2.2);
            this.hndl.yp = plot( this.y_true(1,:,t,rep), this.y_true(2,:,t,rep), 'bo');
            
            this.hndl.xt = draw_traj_nx([],permute(this.x_true(1:2,:,1:t,rep),[3 1 2 4]),'r:');
            this.hndl.yt = draw_traj_nx([],permute(this.y_true(:,:,1:t,rep),[3 1 2 4]),'b:');
            
            % Draw Estimates
            if( ~isempty( this.x_estm ) )
                this.hndl.xe = draw_traj_nx([],permute(this.x_estm(1:2,:,1:t,rep),[3 1 2 4]),'c:');
                this.hndl.xcov = draw_covariances_nx([],this.x_estm(1:2,:,t,rep),this.x_cov(1:2,1:2,:,t,rep));
            end
            if( ~isempty( this.y_estm ) )
                this.hndl.ye = draw_traj_nx([],permute(this.y_estm(:,:,1:t,rep),[3 1 2 4]),'g:');
                this.hndl.ycov = draw_covariances_nx([],this.y_estm(1:2,:,t,rep),this.y_cov(1:2,1:2,:,t,rep),'g');
            end
            
            
            for r = 1:prm.num_x
                this.hndl.fov(r) = draw_fov_nx([],this.x_true(:,r,t,rep),prm.fov,prm.r_sense);
            end

            set(gca,'fontsize',14);
            title(sprintf('Time Step: %d',t));
            xlabel('x [m]','FontSize',14);
            ylabel('y [m]','FontSize',14);            
        end
        
        function update_viz(this,prm,rep,t)
            
            if(isfield(prm,'MAP'))
                set(this.hndl.map,'cdata',(this.vis_map{t,rep}.map|prm.MAP.map).');
            else
                set(this.hndl.map,'cdata',(this.vis_map{t,rep}.map).');
            end

            this.hndl.xp = draw_pose_nx(this.hndl.xp,this.x_true(:,:,t,rep),'r',2.2);
            set(this.hndl.yp,'xdata',this.y_true(1,:,t,rep),'ydata', this.y_true(2,:,t,rep));
            
            this.hndl.xt = draw_traj_nx(this.hndl.xt,permute(this.x_true(1:2,:,1:t,rep),[3 1 2 4]),'r:');
            this.hndl.yt = draw_traj_nx(this.hndl.yt,permute(this.y_true(:,:,1:t,rep),[3 1 2 4]),'b:');
            
            % Draw Estimates
            if( ~isempty( this.x_estm ) )
                this.hndl.xe = draw_traj_nx(this.hndl.xe,permute(this.x_estm(1:2,:,1:t,rep),[3 1 2 4]),'c:');
                this.hndl.xcov = draw_covariances_nx(this.hndl.xcov,this.x_estm(1:2,:,t,rep),this.x_cov(1:2,1:2,:,t,rep));
            end
            if( ~isempty( this.y_estm ) )
                this.hndl.ye = draw_traj_nx(this.hndl.ye,permute(this.y_estm(:,:,1:t,rep),[3 1 2 4]),'g:');
                this.hndl.ycov = draw_covariances_nx(this.hndl.ycov,this.y_estm(1:2,:,t,rep),this.y_cov(1:2,1:2,:,t,rep),'g');
            end
            
            for r = 1:prm.num_x
                this.hndl.fov(r) = draw_fov_nx(this.hndl.fov(r),this.x_true(:,r,t,rep),prm.fov,prm.r_sense);
            end
            
            title(sprintf('Time Step: %d',t));            
        end
    end
        
    methods (Static)        
        function vis_map = remove_visible_vis(vis_map,x,r_sense,fov,MAP)
            % x = num_x x 3 = robot states
            num_x = size(x,1);
            [im,jm] = find(vis_map.map);
            if( ~isempty(im) )
                vis_y = cells2meters([im,jm],vis_map.min.',vis_map.res.');
                for rr = 1:num_x
                    if( nargin < 5 )
                        det_y = detectable2D(vis_y,x(rr,:),r_sense,fov);
                    else
                        %det_y = detectable2D(vis_y,x(rr,:),r_sense,fov);
                        det_y = visible2D(x(rr,:),vis_y,r_sense,fov,MAP);
                    end
                    
                    det_im = im(det_y);
                    det_jm = jm(det_y);
                    lindix = sub2ind_nx(vis_map.size(1),det_im,det_jm);
                    vis_map.map(lindix) = false;
                end
            end
            
            %if( ~isempty(im) )
            %    vis_y = cells2meters([im,jm],vis_map.min.',vis_map.res.');    
            %    if( nargin < 5 )
            %        det_y = detectable2D(vis_y,x,r_sense,fov,true);
            %    else
            %        det_y = detectable2D(vis_y,x,r_sense,fov,true);
            %        %det_y = visible2D(x,vis_y,r_sense,fov,MAP,true);
            %    end
            %    det_y = any(det_y,2);
            %    
            %    det_im = im(det_y);
            %    det_jm = jm(det_y);
            %    lindix = sub2ind_nx(vis_map.size(1),det_im,det_jm);
            %    vis_map.map(lindix) = false;
            %end
        end
        
        
        function exp_map = remove_visible_exp(exp_map,xmap,ymap,x,r_sense,fov,MAP)
            % x = num_x x 3 = robot states
            exp_y = [xmap(exp_map.map), ymap(exp_map.map)];
            num_x = size(x,1);
            for rr = 1:num_x
                if( ~isempty(exp_y) )
                    if( nargin < 7 )
                        det_y = detectable2D(exp_y,x(rr,:),r_sense,fov);
                    else
                        %det_y = detectable2D(exp_y,x(rr,:),r_sense,fov);
                        det_y = visible2D(x(rr,:),exp_y,r_sense,fov,MAP);
                    end
                    exp_map.map(exp_map.map) = ~det_y;
                    exp_y = [xmap(exp_map.map), ymap(exp_map.map)];
                end
            end
               
            %if( ~isempty(exp_y) )
            %    if( nargin < 5 )
            %        det_y = detectable2D(exp_y,x,r_sense,fov,true);
            %    else
            %        det_y = detectable2D(exp_y,x,r_sense,fov,true);
            %        %det_y = visible2D(x,exp_y,r_sense,fov,MAP,true);
            %    end
            %    det_y = any(det_y,2);
            %    
            %    exp_map.map(exp_map.map) = ~det_y;
            %end        
        end
        
        
        function exp_map = add_frontier(exp_map,vis_map,x,r_sense,fov,MAP)
            % x = num_x x 3 = robot states
            if( nargin < 6 )
                MAP = vis_map;
                MAP.map = false(size(MAP.map));
            end
            
            num_robot = size(x,1);
            for r = 1:num_robot
                
                
                laserScan = laser_sim_nx(x(r,:),MAP,0,r_sense+4,fov,0.01);
                valid = vis_map.map(laserScan.frontierLinIdx);
                
                frontCells = meters2cells(laserScan.frontierCoords(:,valid),exp_map.min,exp_map.res);
                frontIdx = sub2ind_nx(exp_map.size.',frontCells(1,:),frontCells(2,:));
                exp_map.map(frontIdx) = true;
            end
            %exp_y = [exp_map.xmap(exp_map.map), exp_map.ymap(exp_map.map)];
        end
    end
                
    
end