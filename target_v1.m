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
    end

    methods
        function this = target_v1(target_id,initial_v,x,n_time_step, type)
            % Initialization
            % x->1 x 3 (x, y, id)   
            this.target_id = target_id;
            this.v = initial_v;
            this.initial_v = initial_v;
            this.x = zeros(n_time_step+1, 3);
            this.x(1,:) = x;
            % assign ids
            this.x(:, 3) = x(3);
            this.type = type;
            this.n_time_step = n_time_step;
            this.state = 'regular';
        end

        function move(this, t, pos_r)
            cur_x = this.x(t, 1:2)';
            dist_vec = cur_x - pos_r(:, 1:2)';
            [min_dist_sqr, min_idx] = min(dist_vec(1, :).^2 + dist_vec(2, :).^2);
            min_dist = sqrt(min_dist_sqr);
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
                    gamma = pi/2 * (this.target_id-1);
                    this.x(t+1, 1) = this.x(t, 1) - this.v * sin(gamma + ang_v*t); %do notihing.
                    this.x(t+1, 2) = this.x(t, 2) + this.v * cos(gamma + ang_v*t);
                elseif strcmp(this.type, 'rect')
                    rep = 10;
                    one_rep_time = this.n_time_step / rep;
                    if mod(t, one_rep_time) < one_rep_time / 4
                        this.x(t+1, 1) = this.x(t, 1) + this.v;
                        this.x(t+1, 2) = this.x(t, 2);
                    elseif mod(t, one_rep_time) >= one_rep_time / 4 && mod(t, one_rep_time) < one_rep_time / 2
                        this.x(t+1, 1) = this.x(t, 1);
                        this.x(t+1, 2) = this.x(t, 2) + this.v;
                    elseif mod(t, one_rep_time) >= one_rep_time / 2 && mod(t, one_rep_time) < 3/4 * one_rep_time
                        this.x(t+1, 1) = this.x(t, 1) - this.v;
                        this.x(t+1, 2) = this.x(t, 2);
                    else
                        this.x(t+1, 1) = this.x(t, 1);
                        this.x(t+1, 2) = this.x(t, 2) - this.v;
                    end
                elseif strcmp(this.type, 'random')
                    theta = rand*2*pi;
                    this.x(t+1, 1:2) = this.x(t, 1:2) + rand*this.v*[cos(theta) sin(theta)];
                elseif strcmp(this.type, 'triangular')
                    this.x(t+1, :) = this.x(t, :);
                elseif strcmp(this.type, 'zigzag')

                elseif strcmp(this.type, 'vertical')
                    this.x(t+1, 1) = this.x(t, 1);
                    this.x(t+1, 2) = this.x(t, 2) + this.v;
                elseif strcmp(this.type, 'horizontal')
                    this.x(t+1, 1) = this.x(t, 1) + this.v;
                    this.x(t+1, 2) = this.x(t, 2);
                else
                    error('unseen type.')
                end
            else 
                % escape mode, double speed and escape.
                escape_dir = dist_vec(:, min_idx) / min_dist;
                this.x(t+1, 1:2) = (this.x(t, 1:2)' + escape_dir * this.v)';
            end
            
        end

        function x = get_x(this,t)
            x = this.x(t, :);
        end
    end
end
