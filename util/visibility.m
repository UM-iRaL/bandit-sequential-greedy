function valid = visibility(x, y, r_sense, fov)
    % x: 3 x 1
    % y: 2 x 1

    if size(x, 1) ~= 3 || size(y, 1) ~= 2 
        error('dimensions mismatch');
    end

    range = norm(x(1:2) - y);
    bearing = bearing_nx(x(1), x(2), y(1), y(2));


    valid = (range < r_sense && abs(restrict_angle(bearing-x(3))) <= fov/2);
end