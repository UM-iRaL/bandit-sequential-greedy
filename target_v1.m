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
        function this = target_v1(v,x,n_time_step, type)
            % Initialization
            % x->1 x 3 (x, y, id)
            
            
            this.v = v;
            this.x = zeros(n_time_step+1, 3);
            this.x(1,:) = x;
            % assign ids
            this.x(:, 3) = x(3);
            this.type = type;
            this.n_time_step = n_time_step;
            if this.type == 'circle'
                ang_v = 12*pi/n_time_step;
                radius = this.v / ang_v;                
                origin = zeros(1, 2);
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
            if strcmp(this.type,'circle')
                ; %do notihing.
            elseif strcmp(this.type, 'rect')              
                this.x(t+1, :) = this.x(t, :);
            elseif strcmp(this.type, 'random')
                theta = rand*2*pi;
                this.x(t+1, 1:2) = this.x(t, 1:2) + rand*this.v*[cos(theta) sin(theta)];
            elseif strcmp(this.type, 'triangular')
                this.x(t+1, :) = this.x(t, :);
            elseif strcmp(this.type, 'zigzag')

            else
                error('unseen type.')
            end
        end

        function x = get_x(this,t)
            x = this.x(t, :);
        end
    end
end