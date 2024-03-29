# fox_robotic_gadget/rob_dyn_control 
This is a Framework for Modeling Identification and Control of Robot Dynamics.  
## Dependencies  
[robotics-toolbox-matlab](https://github.com/petercorke/robotics-toolbox-matlab) is used for robot states display and comparison.  
## Index  
- ModelManipulatorU.m: The main implementation of this project (Make sure the path directory of "ModelManipulatorU.m" can be found when we trying to run any program in this project).  
- get_model_params.m: The setting of main parameters of the project.  
- dymainc_identify_main.m: An example of how to use the "ModelManipulatorU.m".  
- p560akb_dyn_hbmat.m: Base regressor matrix of the manipulator which can be generated by [SymPyBotics](https://github.com/cdsousa/SymPyBotics). We have provided an example at [get_code_hbase_func('matlab')](https://github.com/foxchys/fox_robotic_gadget/blob/master/rob_dyn_sympybotics/Two_link_dyn_iden.py#L125).  
- p560akb_dyn_hmat.m: Regressor matrix of the manipulator which can also be generated by SymPyBotics. We have also provided an example at [get_code_h_func('matlab')](https://github.com/foxchys/fox_robotic_gadget/blob/master/rob_dyn_sympybotics/Two_link_dyn_iden.py#L67).  
- test_real_robot: An example for identification with a real manipulator.  
- admittance_control_dyn: An example for admittance control of a manipulator.  
## Code Example  
Define a robot.  
```
rob_model = ModelManipulatorU('param_func_hd', @get_model_params);
```  
### Optimal robot excitation trajectories  
A robot excitation which has been optimized according to the condition number of regressor matrix.  
```
[exciting_trajectory, exciting_trajectory_period_length,...
    exciting_trajectory_optimal_val]= rob_model.compute_exciting_trajectory(...
    'solve_options', exticiting_trajectory_compute_method,...
    'min_period_length', exticiting_trajectory_min_period_length,...
    'max_period_length', exticiting_trajectory_max_period_length,...
    'use_base_regressor_mat', use_base_dyn_regressor_mat);
```  
The excitation trajectory for each joint is a finite Fourier series. The matlab function [fmincon()](https://www.mathworks.com/help/optim/ug/fmincon.html) is used for the finding of the optimal condition number. The results should be as shown in the figure below, otherwise we should try again (the excitation trajectory might not be good enough).  
![solve_exciting_trajectory.jpg](https://raw.githubusercontent.com/foxchys/fox_robotic_gadget/master/rob_dyn_control/pictures/solve_exciting_trajectory.jpg)  
The `exciting_trajectory_optimal_val` is also a measurement to the quality of the excitation trajectory. The joint limits of the trajectory should also be checked to avoid self collisions.  
For a real robot, we could use the program in "fox_robotic_gadget/rob_dyn_control/test_real_robot/dyn_exciting_trajectory_main.m" Simulation animation is helpful for the trajectory check.  
```
rob_model.rob_model_proterties_.basic_proterties.plot(traj_2plot)
```
The animation should be as shown in the figure below:  
![trajectory_animation.gif](https://raw.githubusercontent.com/foxchys/fox_robotic_gadget/master/rob_dyn_control/pictures/trajectory_animation.gif)  
We could also check the joint limits by ploting the trajectory.  
```
plot(traj_exciting_points(:, j));
```  
The graph should be as shown in the figure below:  
![trajectory_plot.jpg](https://raw.githubusercontent.com/foxchys/fox_robotic_gadget/master/rob_dyn_control/pictures/trajectory_plot.jpg)  
### Robot identification  
Run the excitation trajectory with a robot and read the joints states. The joint position can be direct read from the joint position encoder. Velocity and acceleration could be obtained by numerical differentiation. 
```
traj_rob_q_dq_ddq(1:traj_data_num-1,rob_dof+l) = diff(traj_rob_q_dq_ddq(:,l))/dt;
```   
Digital filter is needed for the joint state data.  
```
traj_rob_q_dq_ddq(:, z) = filtfilt(digital_filter_obj, traj_rob_q_dq_ddq(:, z) );
```  
The least squares method is used for robot identification.
```
dyn_base_param = rob_model.identify_dyn_params_ls(traj_tor,...
    traj_rob_q_dq_ddq(:,1:rob_dof),...
    traj_rob_q_dq_ddq(:,rob_dof+1:2*rob_dof),...
    traj_rob_q_dq_ddq(:,2*rob_dof+1:3*rob_dof),...
    'use_base_regressor_mat', use_base_dyn_regressor_mat);
```  
Where `dyn_base_param` is base dynamic parameters which can be used to compute the joint torque.  
```
tor_regressor_mat = rob_model.compute_joint_tor_regressor_mat(...
    traj_rob_q_dq_ddq(:,1:rob_dof),...
    traj_rob_q_dq_ddq(:,rob_dof+1:2*rob_dof),...
    traj_rob_q_dq_ddq(:,2*rob_dof+1:3*rob_dof),...
    'use_base_dyn_params', use_base_dyn_regressor_mat,...
    'dyn_params_regressor', dyn_base_param);
```  
The results are compared with "robotics-toolbox-matlab".  
```
traj_tor = rob_model.compute_joint_tor_theoretically(...
    traj_rob_q_dq_ddq(:,1:rob_dof),...
    traj_rob_q_dq_ddq(:,rob_dof+1:2*rob_dof),...
    traj_rob_q_dq_ddq(:,2*rob_dof+1:3*rob_dof));
```  
For the robots in simulation, we could get lower error.  
![joint_torque.jpg](https://raw.githubusercontent.com/foxchys/fox_robotic_gadget/master/rob_dyn_control/pictures/joint_torque.jpg)  
For the real robot, we could use the program in "fox_robotic_gadget/rob_dyn_control/test_real_robot/dyn_identify_main.m".  
![traj_34_21.svg](https://raw.githubusercontent.com/foxchys/fox_robotic_gadget/master/rob_dyn_control/pictures/traj_34_21.svg)    
We use the electric current instead of measuring the joint torque (if we don't have joint torque sensor).  
### Admittance control  
The torque required to drive the manipulator can be estimated by the dynamic model. The estimate of the external force can be obtained by making a difference between the actual driving torque and the dynamic model calculation. An admittance model is used to implement the soft interaction response of the manipulator.  
Run "rob_dyn_control/admittance_control_dyn/admittance_control_dyn.slx" to obtain simulation data.  
Run "rob_dyn_control/admittance_control_dyn/plot_rob_sim_traj.m" to get the simulation animation.  
![admittance_control.gif](https://raw.githubusercontent.com/foxchys/fox_robotic_gadget/master/rob_dyn_control/pictures/admittance_control.gif)

## Reference
- Swevers, Jan, et al. "Optimal robot excitation and identification." IEEE transactions on robotics and automation 13.5 (1997): 730-740.  
- Ott, Christian, Ranjan Mukherjee, and Yoshihiko Nakamura. "Unified impedance and admittance control." 2010 IEEE international conference on robotics and automation. IEEE, 2010.  

## License  
This toolbox is released under GNU AGPL license.