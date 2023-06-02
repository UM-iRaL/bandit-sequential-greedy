# Bandit Submodular Maximization for Multi-Robot Coordination in Unpredictable and Partially Observable Environments

Implementation codes (under development) of simulation of the paper [Bandit Submodular Maximization for Multi-Robot Coordination in Unpredictable and Partially Observable Environments](https://arxiv.org/abs/2305.12795).

from Zirui Xu, Xiaofeng Lin and Vasileios Tzoumas at the University of Michigan, Ann Arbor.

## How to use the repo:
We provide codes of one of our scenarios (2 robots vs. 3 non-adversarial targets), please refer to the paper and codes for details.
- run ```main.m``` 


To change the number of robots or targets or type of targets, change parameters in ```main.m```:
```matlab
    num_robot % number of robots
    num_tg % number of targets
    type_tg % type of targets ("normal" or "adversarial")
```

To modify settings of robots and targets, change parameters in function ``` scenarios_settings.m```. All variables should have matched dimensions:
```matlab
    v_robot  % speed of robots
    r_senses % sensing range of robots
    fovs % field of view in degree
    v_tg % speed of targets
    yaw_tg % initial yaw angles of targets
    motion_tg % type of motion of targets (circle, straight)
    x_true_init % initial pose of robots
    tg_true_init % initial pose of targets
```


## Acknowledgement
We would like to thank Professor [Nikolay Atanasov](https://natanaso.github.io/) for giving his advice and codes.
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