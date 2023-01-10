classdef robot_nx < handle
    properties (Constant = true)
        % Vehicle
        T = 0.5;            % sampling period
        stdev_v = 0.1;     % motion noise
        stdev_w = deg2rad(5);
        
        % Sensor 
        fov = deg2rad(94); %deg2rad(94);     % deg2rad(124);
        r_sense = 10;          % meter
        
        % Detection model
        %p0 = [0.9474; 0.8797; 0.9173; 1.0000; 0.8872];
        p0 = 0.90*ones(5,1);
        v0 = 20.5*ones(5,1);
        m0 = 3*ones(5,1);
        
        % Bearing Model
        b_sigma = deg2rad(0.001);
        % To stop bearing noise let b_sigma = 0;
        
        % Range Model
        r_sigma = 0.15;
        % To stop range noise let r_sigma = 0;        
        
        
        % DATA ASSOCIATION:
        da_known = true;
        % MISSED DETECTIONS
        miss_det = false;
        % CLUTTER
        lambda = 0;     % To enable clutter lambda > 0
        
        
        % Class Model
        % Confusion matrix ( hypothesized x true )
        num_class = 5;%2;
        num_score = 11;%28;
        
        meters2pix = 0.003;     % meters per pixel
        cx = 360; cy = 180;     % center of image in pixels
        
        Cmat = [0.7634    0.0410    0.1339    0.0725    0.3089
            0.0229    0.7705    0.0551    0.0145    0.0244
            0.0305    0.0492    0.6378    0.0217    0.0488
            0.0611    0.0983    0.0630    0.8116    0.0244
            0.1221    0.0410    0.1102    0.0797    0.5935];
        % To stop class confusion let Cmat = eye(5);
        
    end
    properties (SetAccess = protected, GetAccess = protected)
        x = [0;0;0];    % robot pose
    end
    properties (SetAccess = protected, GetAccess = public)
        params;            
    end
    
    methods
        function obj = robot_nx(x0)
            if(nargin < 1)
                x0 = [0;0;0];
            end
            obj.set_x(x0(:));
            
            % params
            obj.params.T = obj.T;
            obj.params.stdev_v = obj.stdev_v;
            obj.params.stdev_w = obj.stdev_w;

            obj.params.r_sense = obj.r_sense;
            obj.params.fov = obj.fov;
            obj.params.b_sigma = obj.b_sigma;
            obj.params.r_sigma = obj.r_sigma;
            
            obj.params.p0 = obj.p0;
            obj.params.v0 = obj.v0;
            obj.params.m0 = obj.m0;
            obj.params.fpd = @(d,c) obj.p0(c) .* exp(-abs(obj.m0(c)-d)./obj.v0(c));
            
            obj.params.da_known = obj.da_known;
            obj.params.miss_det = obj.miss_det;
            obj.params.clutter_rate = obj.lambda;
            
            
            obj.params.num_class = obj.num_class;
            obj.params.num_score = obj.num_score;
            obj.params.Cmat = obj.Cmat;
            data = load('pscymat.mat');
            obj.params.pscymat = data.pscymat; % num_score x num_class x num_class
            %obj.params.pc = get_pc(obj);
            %obj.params.ps = get_ps(obj);
            
        end
        
        function set_x(obj,x0)
            if(nargin < 2)
                x0 = [0;0;0];
            end
            obj.x = x0(:);
        end
        
        function x = get_x(obj)
            x = obj.x;
        end
        
        function move(obj,u,smm)
            if(abs(sum(u)) >= 0.1)
                % add noise
                u = u + [obj.stdev_v;obj.stdev_w].*randn(2,1);
            end
            
            if( nargin < 3)
                obj.x = dd_motion_model(obj.x,u,obj.T);
            else
                obj.x = smm(obj.x,u,obj.params);
            end
        end
        
        function z = sense(obj,map,som)
            % map = num_obj x 3 = (x,y,class)
            % som = @generate_msrmnt
            if( nargin < 3)
                z = generate_msrmnt(map,obj.x.',obj.params);
            else
                z = som(map,obj.x.',obj.params);
            end
        end
        
        function z = senseMAP(obj,map,MAP)
            % map = num_obj x 3 = (x,y,class)
            z = generate_msrmnt(map,obj.x.',obj.params,MAP);
        end
    end
end