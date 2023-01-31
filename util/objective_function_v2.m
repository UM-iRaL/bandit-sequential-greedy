function obj_sum = objective_function_v2(robot_states, target_states, r_sense, fov)
% param robot_states: 3 x num_rob
% param target_states: 2 x num_tar
% return obj: objective function

    num_tar = size(target_states, 2);
    num_rob = size(robot_states, 2);
    map_size = 200;
    d_obj = map_size * ones(2, num_tar);
%     obj = -sum(d_obj, 1).^2;
%     obj = (-num_rob^2)*(map_size^2) * ones(num_tar, 1);

    if num_rob ~= 0
        for i = 1:num_tar
            target_state = target_states(:, i);
            for j = 1:num_rob            
                % check if robot j can see this target
                robot_state = robot_states(:, j);    
                if visibility(robot_state, target_state, r_sense, fov)
                    % target i is observed by robot j
                    d_obj(j, i) =  norm(robot_state(1:2) - target_state);
%                     obj(i) = obj(i) - 1 / (d^(1/100)+1);

%                     obj(i) = obj(i) * 1 / (d+1);
                    %obj(i) = obj(i) - d;
                end
            end
        end
    end
    
%     obj = -sum(d_obj.^(1/5), 1);
    obj = -sum(d_obj, 1).^5;
%     obj = -6.^sum(d_obj, 1);

    
    obj_sum = sum(obj);
end