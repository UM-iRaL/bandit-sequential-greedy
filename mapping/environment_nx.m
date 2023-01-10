classdef environment_nx < handle
    properties (SetAccess = protected, GetAccess = public)
        % Environment
        dev=65;             % environment size
        num_obj = 200;       % number of objects
        num_class = 5;
    end
    properties (SetAccess = protected, GetAccess = private)
        map = [];           % list of landmarks
    end
    methods
        function obj = environment_nx(y)
            if nargin < 1
                % Generate Map
                obj.reset();
            else
                obj.set_map(y);
            end
        end
        
        function reset(obj)
            obj.map = zeros(obj.num_obj,3);
            obj.map(:,1:2) = -obj.dev + 2*obj.dev*rand(obj.num_obj,2);
            obj.map(:,3) = randi(obj.num_class,obj.num_obj,1);
        end
        
        function y = get_map(obj)
            y = obj.map;
        end
        
        
        function set_map(obj,y)
          %
          if size(y,2) ~= 3
              error('[environment_nx]: y should be num_y x 3');
          end
          obj.num_obj = size(y,1);
          obj.map = y;
          
          minx = abs(min(y(:,1)));
          maxx = abs(max(y(:,1)));
          miny = abs(min(y(:,2)));
          maxy = abs(max(y(:,2)));
          obj.dev = max([minx, maxx, miny, maxy]);
          
          obj.num_class = numel(unique(y(:,3)));
        end
    end
end
