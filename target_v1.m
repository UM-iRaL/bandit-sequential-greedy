classdef target_v1 < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties (SetAccess = protected, GetAccess = public)
        v;
        initial_v;
        x;
        type;
        n_time_step;
        state;
        target_id;
        radius;
        dT;
        all_min_dist;
    end
    methods
        function this = target_v1(target_id, initial_v, x, yaw,n_time_step, type, dT)
            % Initialization
            % x->1 x 3 (x, y, id)   
            this.target_id = target_id;
            this.v = initial_v;
            this.initial_v = initial_v;
            this.x = zeros(n_time_step+1, 4);
            this.x(1,1:2) = x(1:2);
            this.x(1, 3) = yaw;
            % assign ids
            this.x(:, 4) = x(3);
            this.type = type;
            this.n_time_step = n_time_step;
            this.state = 'regular';
            this.dT = dT;
            this.all_min_dist = zeros(n_time_step, 1);
        end
        function min_dist = min_dist_to_robots(this, t, pos_r)
            if size(pos_r, 2) ~= 3
                error('dimensions mismatch');
            end
            cur_x = this.x(t, 1:2)';
            dist_vec = cur_x - pos_r(:, 1:2)';
            [min_dist_sqr, ~] = min(dist_vec(1, :).^2 + dist_vec(2, :).^2);
            min_dist = sqrt(min_dist_sqr);
        end
        function move(this, t, pos_r)
            %Input:
            %       pos_r: num_robot x 3
            min_dist = this.min_dist_to_robots(t, pos_r);
            this.all_min_dist(t) = min_dist;
            % if robots are within certain range, enter escape mode.
            if min_dist < 0
                this.state = 'escape';
                this.v = this.initial_v * 2;
            else
                this.state = 'regular';
                this.v = this.initial_v;
            end
            if strcmp(this.state, 'regular')
                if strcmp(this.type,'circle')
                    ang_v = this.v/80;
                    this.x(t+1, 3) = this.x(t, 3) + ang_v * this.dT;
                    %this.gamma = pi/2 * (this.target_id-1);
                    this.x(t+1, 1:2) = this.x(t, 1:2) + (this.v*this.dT)*[cos(this.x(t, 3)) sin(this.x(t, 3))]; %do nothing.
                    if mod(t-99, 200) == 0
                        %                     this.x(t+1, 1:2) = this.x(t, 1:2) + (this.v*this.dT + 0.5*randn(1))*[cos(this.x(t, 3)) sin(this.x(t, 3))];
                        this.x(t+1, 3) = this.x(t, 3);
                    end
                elseif strcmp(this.type, 'rect')
                    rep = 10;
                    one_rep_time = this.n_time_step / rep;
                    if mod(t, one_rep_time) < one_rep_time / 4
                        this.x(t+1, 1) = this.x(t, 1) + this.v*this.dT;
                        this.x(t+1, 2) = this.x(t, 2);
                    elseif mod(t, one_rep_time) >= one_rep_time / 4 && mod(t, one_rep_time) < one_rep_time / 2
                        this.x(t+1, 1) = this.x(t, 1);
                        this.x(t+1, 2) = this.x(t, 2) + this.v*this.dT;
                    elseif mod(t, one_rep_time) >= one_rep_time / 2 && mod(t, one_rep_time) < 3/4 * one_rep_time
                        this.x(t+1, 1) = this.x(t, 1) - this.v*this.dT;
                        this.x(t+1, 2) = this.x(t, 2);
                    else
                        this.x(t+1, 1) = this.x(t, 1);
                        this.x(t+1, 2) = this.x(t, 2) - this.v*this.dT;
                    end
                elseif strcmp(this.type, 'random')
                    theta = rand*2*pi;
                    this.x(t+1, 1:2) = this.x(t, 1:2) + rand*this.v*this.dT*[cos(theta) sin(theta)];
                elseif strcmp(this.type, 'triangular')
                    this.x(t+1, :) = this.x(t, :);
                elseif strcmp(this.type, 'zigzag')
                    % TODO:
                elseif strcmp(this.type, 'straight')
                    this.x(t+1, 1:2) = this.x(t, 1:2) + (this.v*this.dT)*[cos(this.x(t, 3)) sin(this.x(t, 3))];
                    this.x(t+1, 3) = this.x(t, 3);
%                 this.x(t+1, 3) = this.x(t, 3) + 0.03*randn(1);    
                else
                    error('unseen type.')
                end
            else 
                % escape mode, double speed and escape.
                escape_dir = dist_vec(:, min_idx) / min_dist;
                this.x(t+1, 1:2) = (this.x(t, 1:2)' + escape_dir * this.v)';
            end        
        end

        function x = get_position(this,t)
            % only get x y coordinate.
            x = this.x(t, [true true false true]);
        end
                function set_pose(this, x, t)
            this.x(t, 1:3) = x;
        end

        function set_yaw(this, t, yaw)
            this.x(t, 3) = yaw;
        end

        function set_v(this, v)
            this.v = v;
        end
        
        function pose = get_pose(this, t)
            pose = this.x(t, 1:3);
        end
        function set_type(this, type)
            this.type = type;
        end
    end
end
