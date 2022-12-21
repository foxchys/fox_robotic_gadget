%     This file is part of fox_robotic_gadget/rob_dyn_control.
%     
%     Copyright (C) 2021-2022, by ChyS(foxchys)
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

%% dynamic identify
% identify base dynamic parameters
sampling_rate = 50;% unit Hz
use_base_dyn_regressor_mat = true;

% traj_1 is used for identify traj_2 is used for dyn-params comparative experimental
traj_real_q_dq_tor_1 = load('./data_traj/34s.txt');%34s
traj_real_q_dq_tor_2 = load('./data_traj/21s.txt');
traj_real_q_dq_tor_1(:, 1:12) = traj_real_q_dq_tor_1(:, 1:12)*pi/1800;
traj_real_q_dq_tor_2(:, 1:12) = traj_real_q_dq_tor_2(:, 1:12)*pi/1800;

rob_model = ModelManipulatorU('param_func_hd', @urlike_get_model_params);
rob_dof = length(rob_model.rob_model_proterties_.basic_proterties.links );

dt = 1/sampling_rate; % s
digital_filter_obj = designfilt('lowpassfir', ...
    'PassbandFrequency',0.01,'StopbandFrequency',0.2, ...
    'PassbandRipple',1,'StopbandAttenuation',60, ...
    'DesignMethod','equiripple');

size_traj_1 = size(traj_real_q_dq_tor_1);
size_traj_2 = size(traj_real_q_dq_tor_2);

traj_1_q_dq_ddq_filtered = zeros(size_traj_1(1,1), 3*rob_dof);
traj_1_tor = zeros(size_traj_1(1,1), rob_dof);

traj_2_q_dq_ddq_filtered = zeros(size_traj_2(1,1), 3*rob_dof);
traj_2_tor = zeros(size_traj_2(1,1), rob_dof);

% ---------------filter the data of trajectory----------------------------
for i = 1:rob_dof
    traj_1_q_dq_ddq_filtered(:, i) = filtfilt(digital_filter_obj, traj_real_q_dq_tor_1(:, i) );
    traj_2_q_dq_ddq_filtered(:, i) = filtfilt(digital_filter_obj, traj_real_q_dq_tor_2(:, i) );
    traj_1_tor(:, i) = filtfilt(digital_filter_obj, traj_real_q_dq_tor_1(:, 2*rob_dof+i) );
    traj_2_tor(:, i) = filtfilt(digital_filter_obj, traj_real_q_dq_tor_2(:, 2*rob_dof+i) );
end

% ---------------compute velocity and acceleration----------------------------
for j = 1:rob_dof
    traj_1_q_dq_ddq_filtered(1:size_traj_1(1,1)-1, rob_dof+j) = ...
        diff(traj_1_q_dq_ddq_filtered(:,j))/dt;
    traj_1_q_dq_ddq_filtered(1:size_traj_1(1,1)-2, 2*rob_dof+j) = ...
        diff(traj_1_q_dq_ddq_filtered(:,j), 2)/(dt^2);
    traj_2_q_dq_ddq_filtered(1:size_traj_2(1,1)-1, rob_dof+j) = ...
        diff(traj_2_q_dq_ddq_filtered(:,j))/dt;
    traj_2_q_dq_ddq_filtered(1:size_traj_2(1,1)-2, 2*rob_dof+j) = ...
        diff(traj_2_q_dq_ddq_filtered(:,j), 2)/(dt^2);
end

disp('computing dynamic params......');
dyn_base_param = rob_model.identify_dyn_params_ls(traj_1_tor,...
    traj_1_q_dq_ddq_filtered(:,1:rob_dof),...
    traj_1_q_dq_ddq_filtered(:,rob_dof+1:2*rob_dof),...
    traj_1_q_dq_ddq_filtered(:,2*rob_dof+1:3*rob_dof),...
    'use_base_regressor_mat', use_base_dyn_regressor_mat);
disp('finished...');

%% test dynamic params
% the comparing between dynamic identify result params and theoretically dynamic params
disp('computing the torque based on regressor matrix......');
tor2_based_dyn_model = rob_model.compute_joint_tor_regressor_mat(...
    traj_2_q_dq_ddq_filtered(:,1:rob_dof),...
    traj_2_q_dq_ddq_filtered(:,rob_dof+1:2*rob_dof),...
    traj_2_q_dq_ddq_filtered(:,2*rob_dof+1:3*rob_dof),...
    'use_base_dyn_params', use_base_dyn_regressor_mat,...
    'dyn_params_regressor', dyn_base_param);
disp('finished...');

for k = 1:rob_dof
    subplot(rob_dof, 1, k);
    plot(traj_real_q_dq_tor_2(:, 2*rob_dof+k));
    hold on;
    plot(tor2_based_dyn_model(:, k));
end



