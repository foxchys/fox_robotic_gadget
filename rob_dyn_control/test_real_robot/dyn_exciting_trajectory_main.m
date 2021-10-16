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
% generate exciting trajectory for dynamic identify

%% params setting
% hyper-parameters setting
rob_model = ModelManipulatorU('param_func_hd', @hd_ur_get_model_params);

use_base_dyn_regressor_mat = true; % always set use_base_dyn_regressor_mat = true
exticiting_trajectory_compute_method = optimoptions('fmincon',...
    'Algorithm','interior-point', 'Display', 'iter-detailed');% Algorithm e.g. active-set
control_rate = 1000;% unit Hz
exticiting_trajectory_min_period_length = 10;% unit s
exticiting_trajectory_max_period_length = 60;% unit s
plot_fps = 10;% value 10 is for robotic tool box
rob_dof = length(rob_model.rob_model_proterties_.basic_proterties.links );

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

traj_2plot = zeros(fix(exciting_trajectory_period_length*plot_fps)+1, rob_dof);
traj_exciting_points = zeros(fix(exciting_trajectory_period_length*control_rate)+1, rob_dof);

for i = 1 : rob_dof
    traj_2plot(:, i) = ...
        exciting_trajectory{i}((0:1/plot_fps:exciting_trajectory_period_length))';
    traj_exciting_points(:, i) = ...
        exciting_trajectory{i}((0:1/control_rate:exciting_trajectory_period_length))';
end

rob_model.rob_model_proterties_.basic_proterties.plot(traj_2plot);

figure;
for j = 1:rob_dof
    subplot(rob_dof, 1, j);
    plot(traj_exciting_points(:, j));
end
