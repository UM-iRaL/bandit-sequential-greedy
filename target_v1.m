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
    end

    methods
        function this = target_v1(initial_v,x,n_time_step, type)
            % Initialization
            % x->1 x 3 (x, y, id)                        
            this.v = initial_v;
            this.initial_v = initial_v;
            this.x = zeros(n_time_step+1, 3);
            this.x(1,:) = x;
            % assign ids
            this.x(:, 3) = x(3);
            this.type = type;
            this.n_time_step = n_time_step;
%             if strcmp(this.type,'circle')
%                 ang_v = 12*pi/n_time_step;
%                 radius = this.v / ang_v;                
%                 origin = zeros(1, 2);
%                 origin(1) = this.x(1, 1) - radius;
%                 origin(2) = this.x(1, 2);
%                 for t = 1:this.n_time_step
%                     this.x(t, 1) = origin(1) + radius*cos(ang_v*t);
%                     this.x(t, 2) = origin(2) + radius*sin(ang_v*t);
%                 end
%             end
            this.state = 'regular';
        end

        function move(this, t, pos_r)
            cur_x = this.x(t, 1:2)';
            dist_vec = cur_x - pos_r(1:2, :);
            [min_dist_sqr, min_idx] = min(dist_vec(1, :).^2 + dist_vec(2, :).^2);
            min_dist = sqrt(min_dist_sqr);
            if min_dist < 0
                this.state = 'escape';
                this.v = this.initial_v * 2;
            else
                this.state = 'regular';
                this.v = this.initial_v;
            end
            if strcmp(this.state, 'regular')
                if strcmp(this.type,'circle')
                    ang_v = 12*pi/this.n_time_step;
                    this.x(t+1, 1) = this.x(t, 1) - this.v * sin(ang_v*t); %do notihing.
                    this.x(t+1, 2) = this.x(t, 2) + this.v * cos(ang_v*t);
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

                else
                    error('unseen type.')
                end
            else 
                escape_dir = dist_vec(:, min_idx) / min_dist;
                this.x(t+1, 1:2) = (this.x(t, 1:2)' + escape_dir * this.v)';
            end
            
        end

        function x = get_x(this,t)
            x = this.x(t, :);
        end
    end
end