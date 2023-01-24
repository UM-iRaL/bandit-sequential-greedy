function d = reward_function(obj_tg)
    % obj_tg: num_tg x 1
    d = 0;
    map_size = 200;
    num_tg = size(obj_tg, 1);
    for kk = 1:num_tg
        if obj_tg(kk) == 0 % meaning this target has not been detected by one single robot
            obj_tg(kk) = -1/(2*map_size^2);
        end
        d = d + 1/obj_tg(kk);
    end
    %f(empty)
    obj_empty = -(2*map_size^2)*num_tg;
    %f(A_i)
    d = (d - obj_empty) / (0 - obj_empty);
end