classdef graph_multirobotslam < handle
    properties (SetAccess = protected, GetAccess = public)
        
        num_robot = 1;
        
        % measurements
        z_prior_x = zeros(3,0); % prior measurement history
        z_prior_y = zeros(2,0); % prior measurement history
        z_relpose = zeros(3,0); % relative pose odometry history
        z_vw = zeros(2,0);      % distance and heading odometry history
        z_range = zeros(1,0);   % range measurement history
        z_bearing = zeros(1,0); % bearing measurement history
        z_rangebearing = zeros(2,0); % range and bearing measurement history        
        
        % measurement covariances
        c_prior_x = zeros(3,3,0);
        c_prior_y = zeros(2,2,0);
        c_relpose = zeros(3,3,0);
        c_vw = zeros(2,2,0);
        c_range = zeros(1,1,0);
        c_bearing = zeros(1,1,0);
        c_rangebearing = zeros(2,2,0);
        
        % data association
        d_prior_x = zeros(1,0);      % 1 x num_z = data association of pose priors
        d_prior_y = zeros(1,0);      % 1 x num_z = data association of landmark priors
        d_relpose = zeros(2,0);      % 2 x num_z = data association of odometry constraints
        d_vw = zeros(2,0);           % 2 x num_z = data association of odometry constraints
        d_range = zeros(2,0);        % 2 x num_z = data association of range measurements
        d_bearing = zeros(2,0);      % 2 x num_z = data association of bearing measurements
        d_rangebearing = zeros(2,0); % 2 x num_z = range and bearing data association
        
        % number of measurements stored
        n_px = 0;
        n_py = 0;
        n_rp = 0;
        n_vw = 0;
        n_r = 0;
        n_b = 0;
        n_rb = 0;
        
        % measurement positions in the square-root info matrix
        ii_x = zeros(1,0); jj_x = zeros(1,0);
        ii_y = zeros(1,0); jj_y = zeros(1,0);
        ii_rp1 = zeros(1,0); jj_rp1 = zeros(1,0);
        ii_rp2 = zeros(1,0); jj_rp2 = zeros(1,0);
        ii_vw1 = zeros(1,0); jj_vw1 = zeros(1,0);
        ii_vw2 = zeros(1,0); jj_vw2 = zeros(1,0);
        ii_r1 = zeros(1,0); jj_r1 = zeros(1,0);
        ii_r2 = zeros(1,0); jj_r2 = zeros(1,0);
        ii_b1 = zeros(1,0); jj_b1 = zeros(1,0);
        ii_b2 = zeros(1,0); jj_b2 = zeros(1,0);
        ii_rb1 = zeros(1,0); jj_rb1 = zeros(1,0);
        ii_rb2 = zeros(1,0); jj_rb2 = zeros(1,0);
        
        % estimates
        odom_init_size = 1000;
        meas_init_size = 500;
        x = zeros(3,1000);  n_x = 0;
        y = zeros(2,500);   n_y = 0;
        

        A = [];
        b = [];
        
    end
    
    methods
        function this = graph_multirobotslam(num_robot)
            % cellulize data structure(measurements, covariances...) for each robot.
            if nargin < 1
                num_robot = 1;
            end
            this.num_robot = num_robot;
            
            % measurements
            this.z_prior_x = this.cellulize(this.z_prior_x,num_robot); % prior measurement history
            %this.z_prior_y = this.cellulize(this.z_prior_y,num_robot); % prior measurement history
            this.z_relpose = this.cellulize(this.z_relpose,num_robot); % relative pose odometry history
            this.z_vw = this.cellulize(this.z_vw,num_robot);           % distance and heading odometry history
            this.z_range = this.cellulize(this.z_range,num_robot);     % range measurement history
            this.z_bearing = this.cellulize(this.z_bearing,num_robot); % bearing measurement history
            this.z_rangebearing = this.cellulize(this.z_rangebearing,num_robot); % range and bearing measurement history
            
            % measurement covariances
            this.c_prior_x = this.cellulize(this.c_prior_x,num_robot);
            %this.c_prior_y = this.cellulize(this.c_prior_y,num_robot);
            this.c_relpose = this.cellulize(this.c_relpose,num_robot);
            this.c_vw = this.cellulize(this.c_vw,num_robot);
            this.c_range = this.cellulize(this.c_range,num_robot);
            this.c_bearing = this.cellulize(this.c_bearing,num_robot);
            this.c_rangebearing = this.cellulize(this.c_rangebearing,num_robot);
            
            % data association
            this.d_prior_x = this.cellulize(this.d_prior_x,num_robot);
            %this.d_prior_y = this.cellulize(this.d_prior_y,num_robot);
            this.d_relpose = this.cellulize(this.d_relpose,num_robot);
            this.d_vw = this.cellulize(this.d_vw,num_robot);
            this.d_range = this.cellulize(this.d_range,num_robot);
            this.d_bearing = this.cellulize(this.d_bearing,num_robot);
            this.d_rangebearing = this.cellulize(this.d_rangebearing,num_robot);
            
            % number of measurements stored
            this.n_px = zeros(num_robot,1);
            %this.n_py = zeros(num_robot,1);
            this.n_rp = zeros(num_robot,1);
            this.n_vw = zeros(num_robot,1);
            this.n_r = zeros(num_robot,1);
            this.n_b = zeros(num_robot,1);
            this.n_rb = zeros(num_robot,1);
            
            % measurement positions in the square-root info matrix for each
            % robot
            this.ii_x = this.cellulize(this.ii_x,num_robot); this.jj_x = this.cellulize(this.jj_x,num_robot);
            %this.ii_y = this.cellulize(this.ii_y,num_robot); this.jj_y = this.cellulize(this.jj_y,num_robot);
            this.ii_rp1 = this.cellulize(this.ii_rp1,num_robot); this.jj_rp1 = this.cellulize(this.jj_rp1,num_robot);
            this.ii_rp2 = this.cellulize(this.ii_rp2,num_robot); this.jj_rp2 = this.cellulize(this.jj_rp2,num_robot);
            this.ii_vw1 = this.cellulize(this.ii_vw1,num_robot); this.jj_vw1 = this.cellulize(this.jj_vw1,num_robot);
            this.ii_vw2 = this.cellulize(this.ii_vw2,num_robot); this.jj_vw2 = this.cellulize(this.jj_vw2,num_robot);
            this.ii_r1 = this.cellulize(this.ii_r1,num_robot); this.jj_r1 = this.cellulize(this.jj_r1,num_robot);
            this.ii_r2 = this.cellulize(this.ii_r2,num_robot); this.jj_r2 = this.cellulize(this.jj_r2,num_robot);
            this.ii_b1 = this.cellulize(this.ii_b1,num_robot); this.jj_b1 = this.cellulize(this.jj_b1,num_robot);
            this.ii_b2 = this.cellulize(this.ii_b2,num_robot); this.jj_b2 = this.cellulize(this.jj_b2,num_robot);
            this.ii_rb1 = this.cellulize(this.ii_rb1,num_robot); this.jj_rb1 = this.cellulize(this.jj_rb1,num_robot);
            this.ii_rb2 = this.cellulize(this.ii_rb2,num_robot); this.jj_rb2 = this.cellulize(this.jj_rb2,num_robot);
            
            % estimates
            this.x = this.cellulize(this.x,num_robot);
            this.n_x = zeros(num_robot,1);
        
        end
        
        function add_prior_x(this,r_id,x_id, x0, inv_sqrt_cov)
            this.z_prior_x{r_id}(:,end+1) = x0;
            this.c_prior_x{r_id}(:,:,end+1) = inv_sqrt_cov;
            this.d_prior_x{r_id}(:,end+1) = x_id;

            this.ii_x{r_id}((end+1):(end+9)) = this.n_px(r_id)*3 + [1 2 3 1 2 3 1 2 3];
            this.jj_x{r_id}((end+1):(end+9)) = (x_id-1)*3 + [1 1 1 2 2 2 3 3 3];
            
            this.n_px(r_id) = this.n_px(r_id) + 1; % add 1 measurement to prior x
        end
        
        function add_prior_y(this,y_id, y0, inv_sqrt_cov)
            this.z_prior_y(:,end+1) = y0;
            this.c_prior_y(:,:,end+1) = inv_sqrt_cov;
            this.d_prior_y(:,end+1) = y_id;
            
            this.ii_y((end+1):(end+4)) = this.n_py*2 + [1 2 1 2];
            this.jj_y((end+1):(end+4)) = (y_id-1)*2 + [1 1 2 2];
            
            this.n_py = this.n_py + 1;
        end
        
        function add_odom_relpose(this,r_id,x_id1,x_id2,z,inv_sqrt_cov)
            
            % check if initialization or reallocation is necessary
            this.z_relpose{r_id} = this.condition_vector(this.z_relpose{r_id}, this.n_rp(r_id), this.odom_init_size);
            this.c_relpose{r_id} = this.condition_vector(this.c_relpose{r_id}, this.n_rp(r_id), this.odom_init_size);
            this.d_relpose{r_id} = this.condition_vector(this.d_relpose{r_id}, this.n_rp(r_id), this.odom_init_size);
            this.ii_rp1{r_id} = this.condition_vector(this.ii_rp1{r_id}, 9*this.n_rp(r_id), 9*this.odom_init_size);
            this.jj_rp1{r_id} = this.condition_vector(this.jj_rp1{r_id}, 9*this.n_rp(r_id), 9*this.odom_init_size);
            this.ii_rp2{r_id} = this.condition_vector(this.ii_rp2{r_id}, 9*this.n_rp(r_id), 9*this.odom_init_size);
            this.jj_rp2{r_id} = this.condition_vector(this.jj_rp2{r_id}, 9*this.n_rp(r_id), 9*this.odom_init_size);
            
            
            % add measurement
            last = this.n_rp(r_id)*9;
            this.ii_rp1{r_id}((last+1):(last+9)) = this.n_rp(r_id)*3 + [1 2 3 1 2 3 1 2 3];
            this.jj_rp1{r_id}((last+1):(last+9)) = (x_id1-1)*3 + [1 1 1 2 2 2 3 3 3];
            this.ii_rp2{r_id}((last+1):(last+9)) = this.n_rp(r_id)*3 + [1 2 3 1 2 3 1 2 3];
            this.jj_rp2{r_id}((last+1):(last+9)) = (x_id2-1)*3 + [1 1 1 2 2 2 3 3 3];
            
            this.n_rp(r_id) = this.n_rp(r_id) + 1;
            this.z_relpose{r_id}(:,this.n_rp(r_id)) = z;
            this.c_relpose{r_id}(:,:,this.n_rp(r_id)) = inv_sqrt_cov;
            this.d_relpose{r_id}(1,this.n_rp(r_id)) = x_id1;
            this.d_relpose{r_id}(2,this.n_rp(r_id)) = x_id2;
        end
        
        function add_odom_vw(this,r_id,x_id1,x_id2,z,inv_sqrt_cov)
            % add distance (ev) and heading (ew) change measurement
            % z = [ev;ew];
            
            % check if initialization or reallocation is necessary
            this.z_vw{r_id} = this.condition_vector(this.z_vw{r_id}, this.n_vw(r_id), this.odom_init_size);
            this.c_vw{r_id} = this.condition_vector(this.c_vw{r_id}, this.n_vw(r_id), this.odom_init_size);
            this.d_vw{r_id} = this.condition_vector(this.d_vw{r_id}, this.n_vw(r_id), this.odom_init_size);
            this.ii_vw1{r_id} = this.condition_vector(this.ii_vw1{r_id}, 6*this.n_vw(r_id), 6*this.odom_init_size);
            this.jj_vw1{r_id} = this.condition_vector(this.jj_vw1{r_id}, 6*this.n_vw(r_id), 6*this.odom_init_size);
            this.ii_vw2{r_id} = this.condition_vector(this.ii_vw2{r_id}, 6*this.n_vw(r_id), 6*this.odom_init_size);
            this.jj_vw2{r_id} = this.condition_vector(this.jj_vw2{r_id}, 6*this.n_vw(r_id), 6*this.odom_init_size);
            
            
            % add measurement
            last = this.n_vw(r_id)*6;
            this.ii_vw1{r_id}((last+1):(last+6)) = this.n_vw(r_id)*2 + [1 2 1 2 1 2];
            this.jj_vw1{r_id}((last+1):(last+6)) = (x_id1-1)*3 + [1 1 2 2 3 3];
            this.ii_vw2{r_id}((last+1):(last+6)) = this.n_vw(r_id)*2 + [1 2 1 2 1 2];
            this.jj_vw2{r_id}((last+1):(last+6)) = (x_id2-1)*3 + [1 1 2 2 3 3];
            
            this.n_vw(r_id) = this.n_vw(r_id) + 1;
            this.z_vw{r_id}(:,this.n_vw(r_id)) = z;
            this.c_vw{r_id}(:,:,this.n_vw(r_id)) = inv_sqrt_cov;
            this.d_vw{r_id}(1,this.n_vw(r_id)) = x_id1;
            this.d_vw{r_id}(2,this.n_vw(r_id)) = x_id2;
        end
        
        function add_meas_range(this,r_id,x_id,y_id,z,inv_sqrt_cov)

            % check if initialization or reallocation is necessary
            this.z_range{r_id} = this.condition_vector(this.z_range{r_id}, this.n_r(r_id), this.meas_init_size);
            this.c_range{r_id} = this.condition_vector(this.c_range{r_id}, this.n_r(r_id), this.meas_init_size);
            this.d_range{r_id} = this.condition_vector(this.d_range{r_id}, this.n_r(r_id), this.meas_init_size);
            this.ii_r1{r_id} = this.condition_vector(this.ii_r1{r_id}, 3*this.n_r(r_id), 3*this.meas_init_size);
            this.jj_r1{r_id} = this.condition_vector(this.jj_r1{r_id}, 3*this.n_r(r_id), 3*this.meas_init_size);
            this.ii_r2{r_id} = this.condition_vector(this.ii_r2{r_id}, 2*this.n_r(r_id), 2*this.meas_init_size);
            this.jj_r2{r_id} = this.condition_vector(this.jj_r2{r_id}, 2*this.n_r(r_id), 2*this.meas_init_size);
            
            % add measurement
            last1 = this.n_r(r_id)*3;
            this.ii_r1{r_id}((last1+1):(last1+3)) = this.n_r(r_id) + [1 1 1];
            this.jj_r1{r_id}((last1+1):(last1+3)) = (x_id-1)*3 + [1 2 3];
            last2 = this.n_r(r_id)*2;
            this.ii_r2{r_id}((last2+1):(last2+2)) = this.n_r(r_id) + [1 1];
            this.jj_r2{r_id}((last2+1):(last2+2)) = (y_id-1)*2 + [1 2];
            
            this.n_r(r_id) = this.n_r(r_id) + 1;
            this.z_range{r_id}(:,this.n_r(r_id)) = z;
            this.c_range{r_id}(:,:,this.n_r(r_id)) = inv_sqrt_cov;
            this.d_range{r_id}(1,this.n_r(r_id)) = x_id;
            this.d_range{r_id}(2,this.n_r(r_id)) = y_id;              
        end
        
        
        function add_meas_bearing(this,r_id,x_id,y_id,z,inv_sqrt_cov)

            % check if initialization or reallocation is necessary
            this.z_bearing{r_id} = this.condition_vector(this.z_bearing{r_id}, this.n_b(r_id), this.meas_init_size);
            this.c_bearing{r_id} = this.condition_vector(this.c_bearing{r_id}, this.n_b(r_id), this.meas_init_size);
            this.d_bearing{r_id} = this.condition_vector(this.d_bearing{r_id}, this.n_b(r_id), this.meas_init_size);
            this.ii_b1{r_id} = this.condition_vector(this.ii_b1{r_id}, 3*this.n_b(r_id), 3*this.meas_init_size);
            this.jj_b1{r_id} = this.condition_vector(this.jj_b1{r_id}, 3*this.n_b(r_id), 3*this.meas_init_size);
            this.ii_b2{r_id} = this.condition_vector(this.ii_b2{r_id}, 2*this.n_b(r_id), 2*this.meas_init_size);
            this.jj_b2{r_id} = this.condition_vector(this.jj_b2{r_id}, 2*this.n_b(r_id), 2*this.meas_init_size);
            
            % add measurement
            last1 = this.n_b(r_id)*3;
            this.ii_b1{r_id}((last1+1):(last1+3)) = this.n_b(r_id) + [1 1 1];
            this.jj_b1{r_id}((last1+1):(last1+3)) = (x_id-1)*3 + [1 2 3];
            last2 = this.n_b(r_id)*2;
            this.ii_b2{r_id}((last2+1):(last2+2)) = this.n_b(r_id) + [1 1];
            this.jj_b2{r_id}((last2+1):(last2+2)) = (y_id-1)*2 + [1 2];            
            
            this.n_b(r_id) = this.n_b(r_id) + 1;
            this.z_bearing{r_id}(:,this.n_b(r_id)) = z;
            this.c_bearing{r_id}(:,:,this.n_b(r_id)) = inv_sqrt_cov;
            this.d_bearing{r_id}(1,this.n_b(r_id)) = x_id;
            this.d_bearing{r_id}(2,this.n_b(r_id)) = y_id;
        end
        
        function add_meas_rangebearing(this,r_id,x_id,y_id,z,inv_sqrt_cov)

            % check if initialization or reallocation is necessary
            this.z_rangebearing{r_id} = this.condition_vector(this.z_rangebearing{r_id}, ...
                this.n_rb(r_id), this.meas_init_size);
            this.c_rangebearing{r_id} = this.condition_vector(this.c_rangebearing{r_id}, ...
                this.n_rb(r_id), this.meas_init_size);
            this.d_rangebearing{r_id} = this.condition_vector(this.d_rangebearing{r_id}, ...
                this.n_rb(r_id), this.meas_init_size);
            this.ii_rb1{r_id} = this.condition_vector(this.ii_rb1{r_id}, 6*this.n_rb(r_id), 6*this.meas_init_size);
            this.jj_rb1{r_id} = this.condition_vector(this.jj_rb1{r_id}, 6*this.n_rb(r_id), 6*this.meas_init_size);
            this.ii_rb2{r_id} = this.condition_vector(this.ii_rb2{r_id}, 4*this.n_rb(r_id), 4*this.meas_init_size);
            this.jj_rb2{r_id} = this.condition_vector(this.jj_rb2{r_id}, 4*this.n_rb(r_id), 4*this.meas_init_size);            
            
            
            % add measurement
            last1 = this.n_rb(r_id)*6;
            this.ii_rb1{r_id}((last1+1):(last1+6)) = this.n_rb(r_id)*2 + [1 2 1 2 1 2];
            this.jj_rb1{r_id}((last1+1):(last1+6)) = (x_id-1)*3 + [1 1 2 2 3 3];
            last2 = this.n_rb(r_id)*4;
            this.ii_rb2{r_id}((last2+1):(last2+4)) = this.n_rb(r_id)*2 + [1 2 1 2];
            this.jj_rb2{r_id}((last2+1):(last2+4)) = (y_id-1)*2 + [1 1 2 2];
            
            this.n_rb(r_id) = this.n_rb(r_id) + 1;
            this.z_rangebearing{r_id}(:,this.n_rb(r_id)) = z;
            this.c_rangebearing{r_id}(:,:,this.n_rb(r_id)) = inv_sqrt_cov;
            this.d_rangebearing{r_id}(1,this.n_rb(r_id)) = x_id;
            this.d_rangebearing{r_id}(2,this.n_rb(r_id)) = y_id;        
        end
        

        function linearize(this,estm_x,estm_y)
            % INPUT:
            %   estm_x = cell(num_robot,1) of 3 x num_x matrices
            %   estm_y = 2 x num_y
            
            if nargin < 3
                estm_y = this.get_estimate_y();
            else
                this.set_estimate_y(estm_y);
            end
            
            if nargin < 2
                estm_x = this.get_estimate_x();
            else
                for r = 1:numel(estm_x)
                    this.set_estimate_x(r,estm_x{r});
                end
            end
            
            % preallocate b
            num_z = [sum(this.n_px)*3, this.n_py*2, sum(this.n_rp)*3, sum(this.n_vw)*2, sum(this.n_r), sum(this.n_b), sum(this.n_rb)*2];
            num_z_cum = cumsum(num_z);
            this.b = zeros(num_z_cum(end),1);
            b_ptr = 0;
            
            m_A = size(this.b,1);
            n_A = sum(this.n_x)*3 + this.n_y*2;
            nnz_A = num_z(1)*3 + num_z(2)*2 + num_z(3)*6 + num_z(4)*6 +...
                num_z(5)*5 + num_z(6)*5 + num_z(7)*5;
            ii_A = zeros(nnz_A,1);
            jj_A = zeros(nnz_A,1);
            jj_offset = [0;cumsum(this.n_x*3)];
            A_ptr = 0;
            
            
            % add x priors
            jacX = [];
            if( num_z(1) > 0 )
                x0 = zeros(3,num_z(1)/3);
                ii_offset = [0;cumsum(this.n_px)];
                px_ptr = 0;
                for r_id = 1:this.num_robot
                    if( this.n_px(r_id) > 0)
                        x0(:,px_ptr+1:px_ptr+this.n_px(r_id)) =  estm_x{r_id}(:,this.d_prior_x{r_id});
                        px_ptr = px_ptr + this.n_px(r_id);
                        
                        idx0 = 1:this.n_px(r_id)*9;
                        ii_A(A_ptr+idx0) = this.ii_x{r_id} + ii_offset(r_id)*3;
                        jj_A(A_ptr+idx0) = this.jj_x{r_id} + jj_offset(r_id);
                        A_ptr = A_ptr + this.n_px(r_id)*9;
                    end
                end
                res = cat(2,this.z_prior_x{:}) - x0;
                res(3,:) = restrict_angle(res(3,:)); %Angular errors in [-pi, pi]
                
                jacX = cat(3,this.c_prior_x{:});
                this.b(b_ptr+1:b_ptr+num_z(1)) = mtimesx(jacX,reshape(res,3,1,[]));
                jacX = jacX(:);
                b_ptr = b_ptr + num_z(1);
            end

            % add y priors
            jacY = [];
            if( this.n_py > 0)
                res = this.z_prior_y - estm_y(:,this.d_prior_y);
                this.b(b_ptr+1:b_ptr+this.n_py*2) = mtimesx(this.c_prior_y,reshape(res,2,1,[]));
                b_ptr = b_ptr + this.n_py*2;
                
                jacY = this.c_prior_y(:);
                
                ii_A(A_ptr+1:A_ptr+num_z(2)*2) = num_z_cum(1) + this.ii_y;
                jj_A(A_ptr+1:A_ptr+num_z(2)*2) = sum(this.n_x)*3 + this.jj_y; 
                A_ptr = A_ptr + num_z(2)*2;
            end
 
                
            % add relpose odom
            jacRP = [];
            if( num_z(3) > 0 )
                x1 = zeros(3,num_z(3)/3);
                x2 = zeros(3,num_z(3)/3);
                z = zeros(3,num_z(3)/3);
                c = zeros(3,3,num_z(3)/3);
                ii_offset = [0;cumsum(this.n_rp)];
                rp_ptr = 0;
                for r_id = 1:this.num_robot
                    if( this.n_rp(r_id) > 0)
                        idx0 = 1:this.n_rp(r_id);
                        idx = rp_ptr+idx0;
                        x2(:,idx) = estm_x{r_id}(:,this.d_relpose{r_id}(2,idx0));
                        x1(:,idx) = estm_x{r_id}(:,this.d_relpose{r_id}(1,idx0));
                        z(:,idx) = this.z_relpose{r_id}(:,idx0);
                        c(:,:,idx) = this.c_relpose{r_id}(:,:,idx0);
                        rp_ptr = rp_ptr + this.n_rp(r_id);
                        
                        % Indices
                        idx0 = 1:this.n_rp(r_id)*9;
                        ii_A(A_ptr+idx0) = num_z_cum(2) + this.ii_rp1{r_id}(idx0) + ii_offset(r_id)*3;
                        jj_A(A_ptr+idx0) = this.jj_rp1{r_id}(idx0) + jj_offset(r_id);
                        A_ptr = A_ptr+this.n_rp(r_id)*9;                       
                    end
                end
                for r_id = 1:this.num_robot
                    if( this.n_rp(r_id) > 0)
                        idx0 = 1:this.n_rp(r_id)*9;
                        ii_A(A_ptr+idx0) = num_z_cum(2) + this.ii_rp2{r_id}(idx0) + ii_offset(r_id)*3;
                        jj_A(A_ptr+idx0) = this.jj_rp2{r_id}(idx0) + jj_offset(r_id);
                        A_ptr = A_ptr+this.n_rp(r_id)*9;
                    end
                end
                res = z - odom_relpose(x2,x1);
                res(3,:) = restrict_angle(res(3,:));
                this.b(b_ptr+1:b_ptr+num_z(3)) = mtimesx(c,reshape(res,3,1,[]));
                b_ptr = b_ptr + num_z(3);
                
                % Jacobian
                [J2, J1] = odom_relpose_jacobian(x2,x1);
                J1 = mtimesx(c,J1);
                J2 = mtimesx(c,J2);
                jacRP = [J1(:);J2(:)];            
            end
            
            
            % add vw odom
            jacVW=[];
            if( num_z(4) > 0 )
                x1 = zeros(3,num_z(4)/2);
                x2 = zeros(3,num_z(4)/2);
                z = zeros(2,num_z(4)/2);
                c = zeros(2,2,num_z(4)/2);
                ii_offset = [0;cumsum(this.n_vw)];
                vw_ptr = 0;
                for r_id = 1:this.num_robot
                    if( this.n_vw(r_id) > 0)
                        idx0 = 1:this.n_vw(r_id);
                        idx = vw_ptr+idx0;
                        x2(:,idx) = estm_x{r_id}(:,this.d_vw{r_id}(2,idx0));
                        x1(:,idx) = estm_x{r_id}(:,this.d_vw{r_id}(1,idx0));
                        z(:,idx) = this.z_vw{r_id}(:,idx0);
                        c(:,:,idx) = this.c_vw{r_id}(:,:,idx0);
                        vw_ptr = vw_ptr + this.n_vw(r_id);
                        
                        % Indices
                        idx0 = 1:this.n_vw(r_id)*6;
                        ii_A(A_ptr+idx0) = num_z_cum(3) + this.ii_vw1{r_id}(idx0) + ii_offset(r_id)*3;
                        jj_A(A_ptr+idx0) = this.jj_vw1{r_id}(idx0) + jj_offset(r_id);
                        A_ptr = A_ptr+this.n_vw(r_id)*6;
                    end
                end
                for r_id = 1:this.num_robot
                    if( this.n_vw(r_id) > 0)
                        idx0 = 1:this.n_vw(r_id)*6;
                        ii_A(A_ptr+idx0) = num_z_cum(3) + this.ii_vw2{r_id}(idx0) + ii_offset(r_id)*3;
                        jj_A(A_ptr+idx0) = this.jj_vw2{r_id}(idx0) + jj_offset(r_id);
                        A_ptr = A_ptr+this.n_vw(r_id)*6;
                    end
                end
                res = z - inv_dd_motion_model(x2,x1);
                res(2,:) = restrict_angle(res(2,:));
                this.b(b_ptr+1:b_ptr+num_z(4)) = mtimesx(c,reshape(res,2,1,[]));
                b_ptr = b_ptr + num_z(4);
                
                % Jacobians
                [J2, J1] = inv_dd_motion_model_jacobian(x2,x1);
                J1 = mtimesx(c,J1);
                J2 = mtimesx(c,J2);
                jacVW = [J1(:);J2(:)];
            end

            % add range meas
            jacR=[];
            if( num_z(5) > 0 )
                x1 = zeros(num_z(5),1);
                x2 = zeros(num_z(5),1);
                y1 = zeros(num_z(5),1);
                y2 = zeros(num_z(5),1);
                z = zeros(num_z(5),1);
                c = zeros(1,1,num_z(5));
                ii_offset = [0;cumsum(this.n_r)];
                r_ptr = 0;
                for r_id = 1:this.num_robot
                    if( this.n_r(r_id) > 0)
                        idx0 = 1:this.n_r(r_id);
                        idx = r_ptr+idx0;
                        x1(idx) = estm_x{r_id}(1,this.d_range{r_id}(1,idx0)).';
                        x2(idx) = estm_x{r_id}(2,this.d_range{r_id}(1,idx0)).';
                        y1(idx) = estm_y(1,this.d_range{r_id}(2,idx0)).';
                        y2(idx) = estm_y(2,this.d_range{r_id}(2,idx0)).';
                        z(idx) = transpose(this.z_range{r_id}(:,idx0));
                        c(:,:,idx) = this.c_range{r_id}(:,:,idx0);
                        r_ptr = r_ptr + this.n_r(r_id);
                        
                        
                        % Indices
                        idx0 = 1:this.n_r(r_id)*3;
                        ii_A(A_ptr+idx0) = num_z_cum(4) + this.ii_r1{r_id}(idx0) + ii_offset(r_id);
                        jj_A(A_ptr+idx0) = this.jj_r1{r_id}(idx0) + jj_offset(r_id);                   
                        A_ptr = A_ptr+this.n_r(r_id)*3;           
                    end
                end
                for r_id = 1:this.num_robot
                    if( this.n_r(r_id) > 0)
                        idx0 = 1:this.n_r(r_id)*2;
                        ii_A(A_ptr+idx0) = num_z_cum(4) + this.ii_r2{r_id}(idx0) + ii_offset(r_id);
                        jj_A(A_ptr+idx0) = sum(this.n_x)*3 + this.jj_r2{r_id}(idx0);
                        A_ptr = A_ptr+this.n_r(r_id)*2;
                    end
                end
                res = z - range_nx(x1,x2,y1,y2);
                this.b(b_ptr+1:b_ptr+num_z(5)) = squeeze(c).*res;
                b_ptr = b_ptr + num_z(5);
                
                % Jacobians
                J1 = range_jacobian_x(x1,x2,y1,y2);
                J1 = mtimesx(c,J1);
                J2 = range_jacobian_y(x1,x2,y1,y2);
                J2 = mtimesx(c,J2);
                jacR = [J1(:);J2(:)];
            end
 
                
            % add bearing meas
            jacB=[];
            if( num_z(6) > 0 )
                x1 = zeros(num_z(6),1);
                x2 = zeros(num_z(6),1);
                x3 = zeros(num_z(6),1);
                y1 = zeros(num_z(6),1);
                y2 = zeros(num_z(6),1);
                z = zeros(num_z(6),1);
                c = zeros(1,1,num_z(6));
                ii_offset = [0;cumsum(this.n_b)];
                bm_ptr = 0;                
                for r_id = 1:this.num_robot
                    if( this.n_b(r_id) > 0)
                        idx0 = 1:this.n_b(r_id);
                        idx = bm_ptr + idx0;
                        x1(idx) = estm_x{r_id}(1,this.d_bearing{r_id}(1,idx0)).';
                        x2(idx) = estm_x{r_id}(2,this.d_bearing{r_id}(1,idx0)).';
                        x3(idx) = estm_x{r_id}(3,this.d_bearing{r_id}(1,idx0)).';
                        y1(idx) = estm_y(1,this.d_bearing{r_id}(2,idx0)).';
                        y2(idx) = estm_y(2,this.d_bearing{r_id}(2,idx0)).';
                        z(idx) = transpose(this.z_bearing{r_id}(:,idx0));
                        c(:,:,idx) = this.c_bearing{r_id}(:,:,idx0);
                        bm_ptr = bm_ptr + this.n_b(r_id);
                        
                        
                        % Indices
                        idx0 = 1:this.n_b(r_id)*3;
                        ii_A(A_ptr+idx0) = num_z_cum(5) + this.ii_b1{r_id}(idx0) + ii_offset(r_id);
                        jj_A(A_ptr+idx0) = this.jj_b1{r_id}(idx0) + jj_offset(r_id);                        
                        A_ptr = A_ptr+this.n_b(r_id)*3;                        
                    end
                end
                for r_id = 1:this.num_robot
                    if( this.n_b(r_id) > 0)
                        idx0 = 1:this.n_b(r_id)*2;
                        ii_A(A_ptr+idx0) = num_z_cum(5) + this.ii_b2{r_id}(idx0) + ii_offset(r_id);
                        jj_A(A_ptr+idx0) = sum(this.n_x)*3 + this.jj_b2{r_id}(idx0);
                        A_ptr = A_ptr+this.n_b(r_id)*2;
                    end
                end
                res = restrict_angle(z - bearing_nx(x1,x2,y1,y2) + x3);
                this.b(b_ptr+1:b_ptr+num_z(6)) = squeeze(c).*res;
                b_ptr = b_ptr + num_z(6);
                
                % Jacobians
                J1 = bearing_jacobian_x(x1,x2,y1,y2);
                J1 = mtimesx(c,J1);
                J2 = bearing_jacobian_y(x1,x2,y1,y2);
                J2 = mtimesx(c,J2);
                jacB = [J1(:);J2(:)];
            end

            % add range-bearing meas
            jacRB=[];
            if( num_z(7) > 0 )
                x1 = zeros(num_z(7)/2,1);
                x2 = zeros(num_z(7)/2,1);
                x3 = zeros(num_z(7)/2,1);
                y1 = zeros(num_z(7)/2,1);
                y2 = zeros(num_z(7)/2,1);
                z = zeros(2,num_z(7)/2);
                c = zeros(2,2,num_z(7)/2);
                ii_offset = [0;cumsum(this.n_rb)];
                rb_ptr = 0;
                for r_id = 1:this.num_robot
                    if( this.n_rb(r_id) > 0)
                        idx0 = 1:this.n_rb(r_id);
                        idx = rb_ptr + idx0;
                        x1(idx) = estm_x{r_id}(1,this.d_rangebearing{r_id}(1,idx0)).';
                        x2(idx) = estm_x{r_id}(2,this.d_rangebearing{r_id}(1,idx0)).';
                        x3(idx) = estm_x{r_id}(3,this.d_rangebearing{r_id}(1,idx0)).';
                        y1(idx) = estm_y(1,this.d_rangebearing{r_id}(2,idx0)).';
                        y2(idx) = estm_y(2,this.d_rangebearing{r_id}(2,idx0)).';
                        z(:,idx) = this.z_rangebearing{r_id}(:,idx0);
                        c(:,:,idx) = this.c_rangebearing{r_id}(:,:,idx0);
                        rb_ptr = rb_ptr + this.n_rb(r_id);
                        
                        % Indices
                        idx0 = 1:this.n_rb(r_id)*6;
                        ii_A(A_ptr+idx0) = num_z_cum(6) + this.ii_rb1{r_id}(idx0) + ii_offset(r_id)*2;
                        jj_A(A_ptr+idx0) = this.jj_rb1{r_id}(idx0)+ jj_offset(r_id);                    
                        A_ptr = A_ptr+this.n_rb(r_id)*6;                     
                    end
                end
                for r_id = 1:this.num_robot
                    if( this.n_rb(r_id) > 0)
                        idx0 = 1:this.n_rb(r_id)*4;
                        ii_A(A_ptr+idx0) = num_z_cum(6) + this.ii_rb2{r_id}(idx0) + ii_offset(r_id)*2;
                        jj_A(A_ptr+idx0) = sum(this.n_x)*3 + this.jj_rb2{r_id}(idx0);
                        A_ptr = A_ptr+this.n_rb(r_id)*4;
                    end
                end
                res = z - [range_nx(x1,x2,y1,y2).';(bearing_nx(x1,x2,y1,y2)-x3).'];
                res(2,:) = restrict_angle(res(2,:));
                this.b(b_ptr+1:b_ptr+num_z(7)) = mtimesx(c,reshape(res,2,1,[]));
                b_ptr = b_ptr + num_z(7);
                
                % Jacobians
                J1 = rb_jacobian_x(x1,x2,y1,y2);
                J1 = mtimesx(c,J1);
                J2 = rb_jacobian_y(x1,x2,y1,y2);
                J2 = mtimesx(c,J2);
                jacRB = [J1(:);J2(:)];
            end
            

            % Construct A
            this.A = sparse(ii_A,jj_A,[jacX;jacY;jacRP;jacVW;jacR;jacB;jacRB], m_A, n_A);
            %this.A = sparse2(ii_A,jj_A,[jacX;jacY;jacRP;jacVW;jacR;jacB;jacRB], m_A, n_A);
            % this.A = sparse2([this.ii_x, num_z_cum(1) + this.ii_y, ...
            %     num_z_cum(2) + this.ii_rp1(1:this.n_rp*9), num_z_cum(2) + this.ii_rp2(1:this.n_rp*9),...
            %     num_z_cum(3) + this.ii_vw1(1:this.n_vw*6), num_z_cum(3) + this.ii_vw2(1:this.n_vw*6),...
            %     num_z_cum(4) + this.ii_r1(1:this.n_r*3), num_z_cum(4) + this.ii_r2(1:this.n_r*2),...
            %     num_z_cum(5) + this.ii_b1(1:this.n_b*3), num_z_cum(5) + this.ii_b2(1:this.n_b*2),...
            %     num_z_cum(6) + this.ii_rb1(1:this.n_rb*6), num_z_cum(6) + this.ii_rb2(1:this.n_rb*4)],...
            %     [this.jj_x,this.n_x*3 + this.jj_y,this.jj_rp1(1:this.n_rp*9),this.jj_rp2(1:this.n_rp*9),...
            %     this.jj_vw1(1:this.n_vw*6),this.jj_vw2(1:this.n_vw*6),...
            %     this.jj_r1(1:this.n_r*3),this.n_x*3 + this.jj_r2(1:this.n_r*2),...
            %     this.jj_b1(1:this.n_b*3),this.n_x*3 + this.jj_b2(1:this.n_b*2),...
            %     this.jj_rb1(1:this.n_rb*6),this.n_x*3 + this.jj_rb2(1:this.n_rb*4)],...
            %     transpose([jacX;jacY;jacRP;jacVW;jacR;jacB;jacRB]), m_A, n_A);
            
        end
                    
        
        function solve(this, lambda, max_iter, var, estm_x, estm_y)
            % INPUT:
            %   estm_x = cell(num_robot,1) of 3 x num_x matrices
            %   estm_y = 2 x num_y            
            if nargin < 4
                var = 'xy';
            end
            
            if nargin < 3
                max_iter = 100;
            end
            
            if nargin < 2
                lambda = 0.01;
            end
            
            if nargin > 4
                for r = 1:numel(estm_x)
                    this.set_estimate_x(r,estm_x{r})
                end
            end
            if nargin > 5
                this.set_estimate_y(estm_y)
            end
            
            %current_error=10000000;
            lambda_max=1e7;
            solver_tol = 1e-5;
            
            this.linearize();
            current_error = norm(this.b);
            step_backs=0;
            i=0;
            done=0;
            while ~done
                i=i+1;
                %fprintf('current_error = %f\n',current_error);
                switch lower(var)
                    case 'x'
                        dv = this.solve_iter_x(lambda);
                    case 'y'
                        dv = this.solve_iter_y(lambda);
                    otherwise
                        dv = this.solve_iter(lambda);
                end
                
                if ((norm(dv)<solver_tol)||(i>=max_iter))
                    done=1;
                    %fprintf('dv too small or max_iter reached\n');
                    %fprintf('norm(dv) = %f\n',norm(dv));
                else
                    %tic;
                    this.linearize();
                    %toc;
                    next_error = norm(this.b);
                    if(next_error<=current_error) && (lambda <= lambda_max)
                        done = (current_error-next_error) < 0.0005; % strange this is here
                        %if(done)
                            %fprintf('error change too small\n');
                        %end
                        current_error=next_error;
                        % successful step
                        lambda = max(0.1*lambda,eps);
                    else
                        lambda=10*lambda;
                        step_backs=step_backs+1;
                        %fprintf('step backs %d\n',step_backs);
                    end
                end
            end
            
        end
        
        function dv = solve_iter(this,lambda)
            % lambda = Levenberg-Marquadt parameter
            nVar = sum(this.n_x)*3 + this.n_y*2;
            
            toScale = false;
            if toScale
                diagJacTJac = sum(this.A.^2,1)';
                scaleMat = spdiags(sqrt(lambda*diagJacTJac),0,nVar,nVar);
            else
                scaleMat = sqrt(lambda)*speye(nVar);
            end
            
            AugJac = [this.A; scaleMat];
            AugRes = [this.b;sparse(nVar,1)];
            
            
            % solve normal equations
            dv = AugJac \ AugRes;
            
            % solve via qr factorization 'vector' OR 0
            %[d,R,P] = qr(AugJac,AugRes,0);        % P = colamd(AugJac);
            %dv = R\d;                             % AugJac = AugJac(:,P);
            %dv(P) = dv;                           % R = qr([AugJac,AugRes]);
                                                  % d = R(:,end);
                                                  % R = R(:,1:end-1);
            
            
            % update state vector
            num_allx = sum(this.n_x)*3;
            estm_x = this.get_estimate_x();
            dx = mat2cell( transpose(reshape(dv(1:num_allx),3,[])), this.n_x );
            for r_id = 1:this.num_robot
                estm_x{r_id} = estm_x{r_id} + transpose(dx{r_id});
                this.set_estimate_x(r_id,estm_x{r_id});
            end
            
            
            estm_y = this.get_estimate_y();
            estm_y = estm_y + reshape(dv((num_allx+1):end),2,[]);
            this.set_estimate_y(estm_y);
        end
        
        function dv = solve_iter_y(this,lambda)
            nVar = this.n_y*2;
            Ay = this.A(:,sum(this.n_x)*3+1:end);
            
            toScale = false;
            if toScale
                diagJacTJac = sum(Ay.^2,1)';
                scaleMat = spdiags(sqrt(lambda*diagJacTJac),0,nVar,nVar);
            else
                scaleMat = sqrt(lambda)*speye(nVar);
            end
            AugJac = [Ay; scaleMat];
            AugRes = [this.b;sparse(nVar,1)];
            
            % solve normal equations
            % dv = AugJac \ AugRes;
            [d,R,P] = qr(AugJac,AugRes,0);
            dv = R\d; dv(P) = dv;

            estm_y = this.get_estimate_y();
            estm_y = estm_y + reshape(dv,2,[]);
            this.set_estimate_y(estm_y);
        end
        
        function dv = solve_iter_x(this,lambda)
            nVar = sum(this.n_x)*3;
            Ax = this.A(:,1:nVar);
            
            toScale = false;
            if toScale
                diagJacTJac = sum(Ax.^2,1)';
                scaleMat = spdiags(sqrt(lambda*diagJacTJac),0,nVar,nVar);
            else
                scaleMat = sqrt(lambda)*speye(nVar);
            end
            AugJac = [Ax; scaleMat];
            AugRes = [this.b;sparse(nVar,1)];
            
            % solve normal equations
            %dv = AugJac \ AugRes;
            [d,R,P] = qr(AugJac,AugRes,0);
            dv = R\d; dv(P) = dv;
            
            
            
            estm_x = this.get_estimate_x();
            dx = mat2cell( transpose(reshape(dv(1:num_allx),3,[])), this.n_x );
            for r_id = 1:this.num_robot
                estm_x{r_id} = estm_x{r_id} + transpose(dx{r_id});
                this.set_estimate_x(r_id,estm_x{r_id});
            end
            
        end
        

        %{
        % Solve via matlab
        function [F,J] = errf(this,x)
            estm_x = x(1:3*this.n_x);
            estm_y = x(3*this.n_x+1:end);
            this.linearize(reshape(estm_x,3,[]),reshape(estm_y,2,[]));
            F = -this.b;
            J = this.A;
        end
        
        function solve_matlab(this,estm_x,estm_y)
            if nargin < 3
                estm_y = this.get_estimate_y();
            else
                this.set_estimate_y(estm_y);
            end
            
            if nargin < 2
                estm_x = this.get_estimate_x();
            else
                this.set_estimate_x(estm_x);
            end
            
            h = @(x) this.errf(x);
            % 'DerivativeCheck','on','Algorithm',
            % 'trust-region-reflective', 'TolFun', 1e-8
            opts = optimset('Display','off','Jacobian', 'on');
            % [final_estm, final_SSE, residual_vector, exitflag, output] = lsqnonlin(h, [estm_x(:);estm_y(:)], [], [], opts);
            final_estm = lsqnonlin(h, [estm_x(:);estm_y(:)], [], [], opts);
            
            estm_x = final_estm(1:3*this.n_x);
            estm_y = final_estm(3*this.n_x+1:end);
            
            this.set_estimate_x(reshape(estm_x,3,[]));
            this.set_estimate_y(reshape(estm_y,2,[]));
        end
        %}
        

        % accessors and mutators
        function append_estimate_x(this,r_id,x)
            n = this.n_x(r_id) + size(x,2);
            this.x{r_id} = this.condition_vector(this.x{r_id}, n, this.odom_init_size);
            this.x{r_id}(:,(this.n_x(r_id)+1):n) = x;
            this.n_x(r_id) = n;
        end
        
        function append_estimate_y(this,y)
            n = this.n_y + size(y,2);
            this.y = this.condition_vector(this.y, n, this.odom_init_size);
            this.y(:,(this.n_y+1):n) = y;
            this.n_y = n;            
        end
        
        function set_estimate_x(this,r_id,x)
            this.n_x(r_id) = 0;
            this.append_estimate_x(r_id,x);            
        end
        
        function set_estimate_y(this,y)
            this.n_y = 0;
            this.append_estimate_y(y);             
        end
        
        function x = get_estimate_x(this)
            x = cell(this.num_robot,1);
            for r_id = 1:this.num_robot
                x{r_id} = this.x{r_id}(:,1:this.n_x(r_id));
            end
        end
        
        function y = get_estimate_y(this)
            y = this.y(:,1:this.n_y);
        end
        
                
        function [estm_x_cov,estm_y_cov,Sig] = get_cov(this)
            %estm_x_cov = 3 x 3 x num_robot
            %estm_y_cov = 2 x 2 x num_y
            
            nVar = sum(this.n_x)*3 + this.n_y*2;
            Hess = (this.A.'*this.A);
            Sig = Hess \ speye(nVar);
            
            idx = nVar - this.n_y*2+1: nVar;
            iiY = reshape(idx,2,1,this.n_y);
            iiY = iiY(:,ones(2,1),:);
            iiY = iiY(:);
            
            jjY = idx;
            jjY = jjY(ones(2,1),:);
            jjY = jjY(:);
            
            lindix = sub2ind_nx(nVar,iiY,jjY);
            estm_y_cov = Sig(lindix);
            estm_y_cov = reshape(full(estm_y_cov),2,2,[]);
            
            
            x_end_idx = cumsum(this.n_x(:)*3);
            idx = [x_end_idx.'-2;x_end_idx.'-1;x_end_idx.'];
            idx = transpose(idx(:));
            
            iiX = reshape(idx,3,1,[]);
            iiX = iiX(:,ones(3,1),:);
            iiX = iiX(:);
            
            jjX = idx(ones(3,1),:);
            jjX = jjX(:);
            lindix = sub2ind_nx(nVar,iiX,jjX);
            estm_x_cov = reshape(full(Sig(lindix)),3,3,[]);
        end

        
        function [jC, mC, jmC]  = get_cholimat(this,x_id,y_id)
            % INPUT:
            %   x_id = cell(num_robot,1) of ids (column vector) for each robot
            %   y_id = cell(num_robot,1) of landmark ids to get
            % OUTPUT:
            %   jC = cholesky factor of the info matrix of x_id and all y's
            %   mC = cell(num_robot,1) of cholesky factors for each robot
            %   and associated y's
            %   jmC = cell(num_robot,1) of cholesky factors for 1:k robot
            %   and associated y's for kth robot.
            
            if nargin < 3
                y_id = repmat({transpose(1:this.n_y)},this.num_robot,1);
            end
            if nargin < 2
                x_id = mat2cell(this.n_x,ones(size(this.n_x)));
            end
            
            offset = cumsum(this.n_x);
            for k = 2:this.num_robot
                x_id{k} = x_id{k} + offset(k-1); 
            end            
            
            all_x_id = cat(1,x_id{:});
            not_x_id = setdiff(transpose(1:offset(end)),all_x_id);
            all_x_id_extended = [all_x_id.'*3-2; all_x_id.'*3-1; all_x_id.'*3];
            not_x_id_extended = [not_x_id.'*3-2; not_x_id.'*3-1; not_x_id.'*3];
            num_x_id_extended = numel(all_x_id_extended);
            
            all_y_id = transpose(1:this.n_y);
            all_y_id_extended = offset(end)*3 + [all_y_id.'*2-1;all_y_id.'*2];
            num_y_id_extended = this.n_y*2;
            
            all_xy_id_extended = [all_x_id_extended(:);all_y_id_extended(:)];
            num_xy_id_extended = num_x_id_extended + num_y_id_extended;

            AA = this.A(:,[not_x_id_extended(:);all_xy_id_extended]);
            jC = qr(AA);
            nVar = offset(end)*3 + this.n_y*2;
            jC = jC((nVar-num_xy_id_extended+1):nVar,(nVar-num_xy_id_extended+1):nVar);

            
            % Obtain marginals
            x_idx = 1:num_x_id_extended;
            offset = 0;
            mC = cell(this.num_robot,1);
            % xi, xi-1,..,x1, sorted union of y_id{i},...,y_id{1}
            jmC = cell(this.num_robot,1);   % Obtain joint-marginals
            for k = 1:this.num_robot
                
                tmp_num = numel(x_id{k}) * 3;
                
                k_not_x_id_joint = x_idx(offset+tmp_num+1:end);
                
                k_x_id = x_idx(offset + 1 : offset+tmp_num);
                k_old_x_id = x_idx(1:offset);
                k_not_x_id = [k_old_x_id, k_not_x_id_joint];
                offset = offset + tmp_num;

                k_not_y_id = setdiff(all_y_id,y_id{k});
                k_y_id = num_x_id_extended + [y_id{k}.'*2-1;y_id{k}.'*2];
                k_not_y_id = num_x_id_extended + [k_not_y_id.'*2-1; k_not_y_id.'*2];
                
                k_id = [k_x_id(:);k_y_id(:)];
                idx = (num_xy_id_extended - numel(k_id) + 1):num_xy_id_extended;
                mC{k} = jC(:,[k_not_x_id(:);k_not_y_id(:);k_id]);
                mC{k} = qr(mC{k});
                mC{k} = mC{k}( idx, idx );
                
                if( k == 1)
                    jmC{k} = mC{k};
                else
                    k_x_id_joint = [k_x_id, k_old_x_id];
                    k_id_joint = [k_x_id_joint(:);k_y_id(:)];
                    idx = (num_xy_id_extended - numel(k_id_joint) + 1):num_xy_id_extended;
                    jmC{k} = jC(:,[k_not_x_id_joint(:);k_not_y_id(:);k_id_joint]);
                    jmC{k} = qr(jmC{k});
                    jmC{k} = jmC{k}( idx, idx );
                end
                 
            end            
        end
        %{%}
        
        %{
        function estm_y_cov = get_cov_y(this)
            Hess = (this.A(:,this.n_x*3+1:end).'*this.A(:,this.n_x*3+1:end));
            Sig = Hess \ speye(this.n_y*2);
            
            idx = 1 : this.n_y*2;
            iiY = reshape(idx,2,1,this.n_y);
            iiY = iiY(:,ones(2,1),:);
            iiY = iiY(:);
            
            jjY = idx;
            jjY = jjY(ones(2,1),:);
            jjY = jjY(:);
            
            lindix = sub2ind_nx(this.n_y*2,iiY,jjY);
            estm_y_cov = Sig(lindix);
            estm_y_cov = reshape(full(estm_y_cov),2,2,[]);            
        end
        
        
        function estm_x_imat = get_imat_x(this)
            Hess = (this.A.'*this.A);
            our_idx = this.n_x*3-2:this.n_x*3;
            other_idx = setdiff(1:(this.n_x*3 + this.n_y*2),our_idx);
            estm_x_imat = Hess(our_idx,our_idx) -...
                Hess(our_idx,other_idx)/Hess(other_idx,other_idx)*Hess(other_idx,our_idx);            
        end
        
        function estm_y_imat = get_imat_y(this)
            Hess = (this.A.'*this.A);
            estm_y_imat = Hess(this.n_x*3+1:end,this.n_x*3+1:end)-...
                Hess(this.n_x*3+1:end,1:this.n_x*3)/Hess(1:this.n_x*3,1:this.n_x*3)*Hess(1:this.n_x*3,this.n_x*3+1:end);
        end
        
        function estm_xy_imat = get_imat_xy(this)
            Hess = (this.A.'*this.A);
            H_a = Hess(this.n_x*3-2:end,this.n_x*3-2:end);
            H_d = Hess(1:this.n_x*3-3,1:this.n_x*3-3);
            H_b = Hess(this.n_x*3-2:end,1:this.n_x*3-3);
            estm_xy_imat = H_a - H_b/H_d*H_b';            
        end
        
        function estm_xy_cholimat = get_cholimat_xy(this)
            R = qr(this.A);
            estm_xy_cholimat = R((this.n_x*3-2):(this.n_x*3+this.n_y*2),(this.n_x*3-2):(this.n_x*3+this.n_y*2));
        end
        
        
        function [estm_x_cov,estm_y_cov] = get_cov_v2(this)
            %estm_x_cov = [];
            %estm_y_cov = [];
            Hess = (this.A.'*this.A);
            H_a = Hess(this.n_x*3-2:end,this.n_x*3-2:end);
            H_d = Hess(1:this.n_x*3-3,1:this.n_x*3-3);
            H_b = Hess(this.n_x*3-2:end,1:this.n_x*3-3);
            Hess_xy = H_a - H_b/H_d*H_b';
            
            nVar = this.n_y*2+3;
            Sig_xy = Hess_xy \ speye(nVar);            
            estm_x_cov = full(Sig_xy(1:3,1:3));
            
            idx = 4:nVar;
            iiY = reshape(idx,2,1,this.n_y);
            iiY = iiY(:,ones(2,1),:);
            iiY = iiY(:);
            
            jjY = idx;
            jjY = jjY(ones(2,1),:);
            jjY = jjY(:);
            
            lindix = sub2ind_nx(nVar,iiY,jjY);
            estm_y_cov = Sig_xy(lindix);
            estm_y_cov = reshape(full(estm_y_cov),2,2,[]);
        end
        %}

        
    end
    
    
    methods (Static)
        function v = condition_vector(v,n,init_sz)
            % n = number of elements currently in v
            % init_sz = size to initialize v to
            % make v into size(v)(1:end-1), init_sz
            nd = ndims(v);
            sz = size(v);            
            if(isempty(v))
                v = cat(nd,v,zeros([sz(1:end-1),init_sz]));
            elseif(n >= sz(nd))
                toadd_sz = max(sz(end),n-sz(end));
                v = cat(nd,v,zeros([sz(1:end-1),toadd_sz]));
            end
        end
        
        function c = cellulize(X,n)
            % returns a cell(n,1) with each cell equal to X
            c = repmat({X},n,1);
        end
    end
end