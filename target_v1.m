classdef target_v1 < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties (SetAccess = protected, GetAccess = public)
        v;
        x;
        type;
        n_time_step;
    end

    methods
        function this = target(v,x,n_time_step, type)
            % Initialization
            % x->1 x 3 (x, y, id)
            
            
            this.v = v;
            this.x = zeros(n_time_step, 3);
            this.x(1,:) = x;
            % assign ids
            this.x(:, 3) = x(3);
            this.type = type;
            this.n_time_step = n_time_step;
            if this.type == 'circle'
                ang_v = 2*pi/n_time_step;
                radius = this.v / ang_v;                
                origin = zeors(1, 2);
                origin(1) = this.x(1, 1) - radius;
                origin(2) = this.x(1, 2);
                for t = 1:this.n_time_step
                    this.x(t, 1) = origin(1) + radius*cos(ang_v*t);
                    this.x(t, 2) = origin(2) + radius*sin(ang_v*t);
                end
            end
        end

        function move(this, t)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            if this.type == 'circle'
                ; %do notihing.
            elseif this.type == 'rectangular'
                this.x(t+1, :) = this.x(t, :);
            elseif this.type == 'random'
                this.x(t+1, :) = this.x(t, :);
            elseif this.type == 'triangular'
                this.x(t+1, :) = this.x(t, :);
            elseif this.type == 'zigzag'
            else
                error('unseen type.')
            end
        end

        function x = get_x(this,t)
            x = this.x(t, :);
        end
    end
end