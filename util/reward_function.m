function r = reward_function(obj_tg)
    % Input : 
    % obj_mat: obj_mat(kk) contains kkth target of corresponding
    % sum -1/(r_i.^2) with r being the distance btw kkth target and
    % ith robot that detects kkth target.
    % Output : reward
    d = 0;
    map_size = 500;
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
    r = (d - obj_empty) / (0 - obj_empty);
    if r < 0
        error("reward can't be negative");
    end
end