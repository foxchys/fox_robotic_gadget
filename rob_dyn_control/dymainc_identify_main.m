%     This file is part of fox_robotic_gadget/rob_dyn_control.
%     
%     Copyright (C) 2021, by ChyS(foxchys)
% 
%     This program is free software: you can redistribute it and/or modify
%     it under the terms of the GNU Affero General Public License as published
%     by the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
% 
%     This program is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU Affero General Public License for more details.
% 
%     You should have received a copy of the GNU Affero General Public License
%     along with this program.  If not, see <https://www.gnu.org/licenses/>.
%     
%     ChyS(foxchys): https://github.com/foxchys

%% dynamic identify for manipulator
% dynamic identify for arbitrary dof serial manipulator 

%% params setting
% hyper-parameters setting
rob_model = ModelManipulatorU('param_func_hd', @get_model_params);

use_base_dyn_regressor_mat = true; % always set use_base_dyn_regressor_mat = true
exticiting_trajectory_compute_method = optimoptions('fmincon',...
    'Algorithm','interior-point', 'Display', 'iter-detailed');% Algorithm e.g. 
control_rate = 1000;% unit Hz
exticiting_trajectory_min_period_length = 10;% unit s
exticiting_trajectory_max_period_length = 60;% unit s

digital_filter_obj = designfilt('lowpassfir', ...
    'PassbandFrequency',0.15,'StopbandFrequency',0.2, ...
    'PassbandRipple',1,'StopbandAttenuation',60, ...
    'DesignMethod','equiripple');

%% exciting trajectory
% compute exciting trajectory for identify of dynamic
[exciting_trajectory, exciting_trajectory_period_length,...
    exciting_trajectory_optimal_val]= rob_model.compute_exciting_trajectory(...
    'solve_options', exticiting_trajectory_compute_method,...
    'min_period_length', exticiting_trajectory_min_period_length,...
    'max_period_length', exticiting_trajectory_max_period_length,...
    'use_base_regressor_mat', use_base_dyn_regressor_mat);
disp('optimal_val of the exciting trajectory:');
disp(exciting_trajectory_optimal_val);
% for i = 1:length(exciting_trajectory)
%     plot( (0:1/control_rate:exciting_trajectory_period_length),...
%         exciting_trajectory{i}(...
%         (0:1/control_rate:exciting_trajectory_period_length)) );
%     hold on;
%     % pause(1);
% end

%% dynamic identify
% identify base dynamic parameters

% ---------------get the data of trajectory----------------------------
traj_data_num = fix(control_rate*exciting_trajectory_period_length);

rob_dof = length(rob_model.rob_model_proterties_.basic_proterties.links);
traj_rob_q_dq_ddq = zeros(traj_data_num, 3*rob_dof);
dt = 1/control_rate;

for j = 1:traj_data_num
    for k = 1:rob_dof
        traj_rob_q_dq_ddq(j, k) = exciting_trajectory{k}(j*dt);
    end
end

for l = 1:rob_dof
    traj_rob_q_dq_ddq(1:traj_data_num-1,rob_dof+l) = diff(traj_rob_q_dq_ddq(:,l))/dt;
    traj_rob_q_dq_ddq(1:traj_data_num-2, 2*rob_dof+l) = diff(traj_rob_q_dq_ddq(:,l), 2)/(dt^2);
end

traj_rob_q_dq_ddq = traj_rob_q_dq_ddq(1:traj_data_num-2, :);

for z = (rob_dof+1):(3*rob_dof)
    traj_rob_q_dq_ddq(:, z) = filtfilt(digital_filter_obj, traj_rob_q_dq_ddq(:, z) );
end

% ---------------------------------------------------------------------

% -----get data of torque based on theoretically dynamic params---------
% traj_tor = zeros(traj_data_num-2, rob_dof);

traj_tor = rob_model.compute_joint_tor_theoretically(...
    traj_rob_q_dq_ddq(:,1:rob_dof),...
    traj_rob_q_dq_ddq(:,rob_dof+1:2*rob_dof),...
    traj_rob_q_dq_ddq(:,2*rob_dof+1:3*rob_dof));

disp('computing dynamic params......');
dyn_base_param = rob_model.identify_dyn_params_lls(traj_tor,...
    traj_rob_q_dq_ddq(:,1:rob_dof),...
    traj_rob_q_dq_ddq(:,rob_dof+1:2*rob_dof),...
    traj_rob_q_dq_ddq(:,2*rob_dof+1:3*rob_dof),...
    'use_base_regressor_mat', use_base_dyn_regressor_mat);
disp('finished...');

%% test dynamic params
% the comparing between dynamic identify result params and theoretically dynamic params
disp('computing the torque based on regressor matrix......')
tor_regressor_mat = rob_model.compute_joint_tor_regressor_mat(...
    traj_rob_q_dq_ddq(:,1:rob_dof),...
    traj_rob_q_dq_ddq(:,rob_dof+1:2*rob_dof),...
    traj_rob_q_dq_ddq(:,2*rob_dof+1:3*rob_dof),...
    'use_base_dyn_params', use_base_dyn_regressor_mat);
disp('finished...');

for n=1:rob_dof
    plot(traj_tor(:,n));
    hold on;
    plot(tor_regressor_mat(:,n))
end


