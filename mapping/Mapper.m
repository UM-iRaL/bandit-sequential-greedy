classdef Mapper < handle
    properties (SetAccess = protected, GetAccess = public)
        fgraph; % The factor graph
        reset_time = 100;
        error_struct;
        
        x_ids = []; % mapping from sensor id to slam order
        y_ids = {}; % mapping from landmark id to slam order
        
        fg_estm = {}; % current factor graph estimate
        
    end
    
    methods
        function this = Mapper(num_x,A)
            
            this.fgraph = slam_graph_v5;
             
            % Initialize errors and jacobians
            % type 1: prior
            this.error_struct(1).type = 'unary';
            this.error_struct(1).err = @(z,y) y-z;
            this.error_struct(1).jac = {@(y) repmat(eye(size(y,1)),1,1,size(y,2))};
            
            
            % type 2: sensor odometry
            this.error_struct(2).type = 'binary';
            this.error_struct(2).err =@err_relpose;
            this.error_struct(2).jac = {@(x,y) odom_relpose_jacobian_nx(x,y,1); ...
                                        @(x,y) odom_relpose_jacobian_nx(x,y,2) };
                              
            % type 3: target odometry
            this.error_struct(3).type = 'binary';
            this.error_struct(3).err = @(z,ny,y) (ny - A*y) - z;
            this.error_struct(3).jac = {@(x,y) repmat(eye(size(x,1)),1,1,size(x,2)); ...
                                        @(x,y) repmat(-A,1,1,size(y,2)) };
            %{%}
            % type 4: range-bearing jacobian
            this.error_struct(4).type = 'binary';
            this.error_struct(4).err = @err_rb;
            this.error_struct(4).jac = {@(x,y) rb_jacobian_x(x(1,:).',x(2,:).',y(1,:).',y(2,:).'); ...
                                        @(x,y) rb_jacobian_y(x(1,:).',x(2,:).',y(1,:).',y(2,:).') };
            

            % Initialize ids (TODO: fix this)
            this.x_ids = zeros(num_x,this.reset_time+1);
            this.y_ids = cell(0,this.reset_time+1);
        end
        
        function add_y_prior(this,y_id,y_estm,y_cov,t)
            % y_estm = dim_y x 1
            % y_cov = dim_y x dim_y
            
            curr_t = mod(t-1,this.reset_time)+1;
            dim_y = length(y_estm);
            
            num_y_in_graph = size(this.y_ids,1);
            if( y_id > num_y_in_graph || isempty( this.y_ids{y_id,curr_t} ) )
                % never seen this landmark before
                this.y_ids{y_id,curr_t} = length(this.fg_estm)+1;
            end
            
            % look-up landmark id
            y_id =  this.y_ids{y_id,curr_t};
            % add to graph
            this.fgraph.add_unary(1,this.error_struct(1).err,y_id,dim_y,...
                    this.error_struct(1).jac{1},y_estm, eye(dim_y)/chol(y_cov));
            % update estimate
            this.fg_estm{y_id} = y_estm(:);
            
        end
        
        function add_xy_rangebearing(this,x_id,y_id,z_val,ciV,t,x_val,y_val)
            curr_t = mod(t-1,this.reset_time)+1;
            num_y_in_graph = size(this.y_ids,1);
            if( y_id > num_y_in_graph || isempty( this.y_ids{y_id,curr_t} ) )
                % never seen this landmark before
                this.y_ids{y_id,curr_t} = length(this.fg_estm)+1;
                this.fg_estm{ this.y_ids{y_id,curr_t} } = y_val(:);
            end
            
            if( this.x_ids(x_id,curr_t) == 0 )
                % never seen this sensor before
                this.x_ids(x_id,curr_t) = length(this.fg_estm)+1;
                this.fg_estm{ this.x_ids(x_id,curr_t) } = x_val(:);
            end
            
            y_id = this.y_ids{y_id,curr_t};
            x_id = this.x_ids(x_id,curr_t);

            dim_y = length(this.fg_estm{ y_id });
            dim_x = length(this.fg_estm{ x_id });
            
            this.fgraph.add_binary(4,this.error_struct(4).err,...
                x_id, dim_x, this.error_struct(4).jac{1},...
                y_id, dim_y, this.error_struct(4).jac{2},...
                z_val(:), ciV.' );
        end
        
        function add_y_odom(this,y_id,A,ciW,t)
            % Add Odometry to SLAM
            % Assumes y at time t is in the graph but not at t+1
            
            curr_t = mod(t-1,this.reset_time)+1;
            
            % add new estimate to graph
            y_id_graph = this.y_ids{y_id,curr_t};
            this.y_ids{y_id,curr_t+1} = length(this.fg_estm)+1;
            this.fg_estm{end+1} = A*this.fg_estm{y_id_graph};
            dim_y = size(A,1);
            
            % Add measurement
            % TODO: add jacobian parameters.. to handle time varying A
            this.fgraph.add_binary(3, this.error_struct(3).err,...
                this.y_ids{y_id,curr_t+1}, dim_y, this.error_struct(3).jac{1},...
                this.y_ids{y_id,curr_t}, dim_y, this.error_struct(3).jac{2},...
                zeros(dim_y,1), ciW.');
        end
        
        %{
        function check_reset(this, Logger, t,rep)
            if( mod(t,this.reset_time) == 0 )
                this.optimize();
                this.save_estimates_to_logger(this, Logger, t, rep);
                
                % Reset graph
                num_x = size(Logger.x_true,2);
                num_y = size(Logger.y_estm,2);
                this = Mapper(num_x);
                for k = 1:num_y
                    this.add_y_prior(k,Logger.y_estm(:,k,t,rep),Logger.y_cov(:,:,k,t,rep),t);
                end
            end
        end
        %}
            
        function optimize(this)
            valid_idx = cat(1,this.y_ids{:});
            valid_bool = false(size(this.fg_estm));
            valid_bool(valid_idx) = true;            
            this.fg_estm = this.fgraph.solve(this.fg_estm, 0.01, 100, valid_bool);
        end
        
        function S = get_cov(this)
            S = slam_graph_v5.get_cov(this.fgraph.A);
        end
        
        function save_estimates_to_logger(this, Logger, t, rep)
            S = this.get_cov();
                        
            dim_y = size(Logger.y_estm,1);
            num_y = size(Logger.y_estm,2);
            num_y_in_graph = size(this.y_ids,1);
            
            valid_idx = cat(1,this.y_ids{:});
            valid_bool = false(size(this.fg_estm));
            valid_bool(valid_idx) = true;
            
            vdims = transpose(cellfun(@length,this.fg_estm));
            cvdims = cumsum([0;vdims( valid_bool)]);
            curr_t = mod(t-1,this.reset_time)+1;

            for kk = 1:num_y
                for s = 1:curr_t                    
                    if( ~(kk > num_y_in_graph) && ~isempty( this.y_ids{kk,s} ) )
                        Logger.y_estm(:,kk,t-curr_t+s,rep) = this.fg_estm{this.y_ids{kk,s}};
                        cv_idx = this.fgraph.all_to_lin_idx(this.y_ids{kk,s}); % cv_idx=(valid_idx==y_ids(kk,s));
                        idx = cvdims(cv_idx)+transpose(1:dim_y);
                        Logger.y_cov(:,:,kk,t-curr_t+s,rep) = S(idx,idx);
                    end
                end
            end
            %{
            for kk = 1:num_y
                for s = 1:curr_t
                    if( isKey(Logger.landmark_map{rep}, kk) && ~isempty( this.y_ids{Logger.landmark_map{rep}(kk),s} ) )
                        tmp = this.y_ids{Logger.landmark_map{rep}(kk),s};
                        Logger.y_estm(:,kk,t-curr_t+s,rep) = this.fg_estm{tmp};
                        cv_idx = this.fgraph.all_to_lin_idx(tmp); % cv_idx=(valid_idx==y_ids(kk,s));
                        idx = cvdims(cv_idx)+transpose(1:dim_y);
                        Logger.y_cov(:,:,kk,t-curr_t+s,rep) = S(idx,idx);
                    end
                end
            end
            %}
        end
        
        
        function mC = get_marginal_cholesky(this, y_id_cell, t )
            curr_t = mod(t-1,this.reset_time)+1;
            slam_id_cell = cell(size(y_id_cell));
            for r = 1:length(slam_id_cell)
                slam_id_cell{r} = cat(1,this.y_ids{y_id_cell{r},curr_t});
            end
            mC = this.fgraph.get_choimat(slam_id_cell);
        end

    end    
end

