

## 2 vs 2
params:
**two_vs_two_scene.m**
`tg_true(:,1,1,:) = repmat([-100;0;1],1,num_rep);
`tg_true(:,2,1,:) = repmat([0;-100;2],1,num_rep);`

`v_tg = 0.24`
`v_r = 1`

`x_true(1, 1, :, :) = repmat([-120;0;0],1,num_rep);
`x_true(1, 2, :, :) = repmat([0; -120; pi/2],1,num_rep);`

**bsg_planner_nx_v1**
`learning_const = 2`

**robot_nx**
`% Sensor
`fov = deg2rad(94); %deg2rad(94); % deg2rad(124);
`r_sense = 50; % meter`



