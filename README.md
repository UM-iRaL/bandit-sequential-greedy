# Bandit Submodular Maximization for Multi-Robot Coordination in Unpredictable and Partially Observable Environments

## About
This repository holds the implementation code for the simulation scenario with 2 robots vs. 3 non-adversarial targets that appears in the paper 

Zirui Xu, Xiaofeng Lin, and Vasileios Tzoumas, [``Bandit Submodular Maximization for Multi-Robot Coordination in Unpredictable and Partially Observable Environments,''](https://arxiv.org/abs/2305.12795), Robotics: Science and Systems, 2023.

### Run the simulation
- run ```main.m``` 


### Design new scenarios

To change the number of robots/the number of targets/the type of a target, please modify the following parameters in ```main.m```:
```matlab
    num_robot % number of robots
    num_tg % number of targets
    type_tg % type of targets ("normal" or "adversarial")
```

To modify settings of robots and targets, please change the following parameters in ``` scenarios_settings.m``` (notice all variables should have matching dimensions):
```matlab
    v_robot      % speed of robots
    r_senses     % sensing range of robots
    fovs         % field of view in degree
    v_tg         % speed of targets
    yaw_tg       % initial yaw angles of targets
    motion_tg    % type of motion of targets (circle, straight)
    x_true_init  % initial pose of robots
    tg_true_init % initial pose of targets
```


## Acknowledgement
We thank [Nikolay Atanasov](https://natanaso.github.io/) for sharing the code for [``Decentralized active information acquisition: Theory and application to multi-robot SLAM''](https://natanaso.github.io/ref/Atanasov_ActiveInformationAcquisition_ICRA15.pdf).

## License
The project is licensed under MIT License

## Citation
If you have an academic use, please cite:

```
@inproceedings{xu2023bandit,
  title={Bandit Submodular Maximization for Multi-Robot Coordination in Unpredictable and Partially Observable Environments},
  author={Xu, Zirui and Lin, Xiaofeng and Tzoumas, Vasileios},
  journal={Robotics: Science and Systems},
  year={2023}
}
```
