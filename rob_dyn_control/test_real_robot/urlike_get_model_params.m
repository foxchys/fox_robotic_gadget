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

function [rob_model_proterties] = urlike_get_model_params()
%GET_MODEL_PARAMS Setting of manipulator proterties
%   Basic proterties of the manipulator 

% dynamic_theoretically__flg = true; use the theoretically dynamic params
% dynamic_theoretically__flg = false; need to indentify the dynamic params
theoretically_dynamic_flg = false;
% dynamic linear regressor matrix function
rob_dyn_hmatrix_func = @urlike_dyn_hmat; 
% linear dynamic params based on regressor-matrix
rob_dyn_params_h = [];
% dynamic base linear regressor matrix function
rob_dyn_hbasematrix_func = @urlike_dyn_hbmat; 
% linear dynamic params based on base-regressor-matrix
rob_dyn_params_baseh = ...
    [];
% gravity reference robot-base-frame type: [x, y, z]
gravity = [0 0 -9.8];



% robot model based on the ur_like_manipulator
%           theta    d        a    alpha    offset
L(1) = Link([  0      0.125        0       0      0], 'modified');
L(2) = Link([  0      0        0      pi/2    0], 'modified');
L(3) = Link([  0      0      -0.300    0      0], 'modified');
L(4) = Link([  0     0.1105  -0.276    0      0], 'modified');
L(5) = Link([  0     0.09      0      pi/2    0], 'modified');
L(6) = Link([  0      0.082    0     -pi/2    0], 'modified');

L(2).offset = -pi/2;
L(4).offset = -pi/2;

% limit [theta_min, theta_max, d_theta_min(angular velocity rad/s),
%        d_theta_max, dd_theta_min, dd_theta_max(angular acceleration rad/s^2)]
joints_limt = [-120*pi/180, 120*pi/180, -40*pi/180, 40*pi/180, -48*pi/180, 48*pi/180;
               -90*pi/180, 90*pi/180, -40*pi/180, 40*pi/180, -24*pi/180, 24*pi/180;
               -100*pi/180, 100*pi/180, -40*pi/180, 40*pi/180, -24*pi/180, 24*pi/180;
               -120*pi/180, 120*pi/180, -40*pi/180, 40*pi/180, -24*pi/180, 24*pi/180;
               -120*pi/180, 120*pi/180, -40*pi/180, 40*pi/180, -24*pi/180, 24*pi/180;
               -120*pi/180, 120*pi/180, -40*pi/180, 40*pi/180, -24*pi/180, 24*pi/180];


%%%%%%---------set theoretically dynamic params-------------------%%%%%%%
%e.g. dynamic params from solidworks
if theoretically_dynamic_flg

    
%     L(1).m = 0;
%     L(2).m = 17.4;
%     L(3).m = 4.8;
%     L(4).m = 0.82;
%     L(5).m = 0.34;
%     L(6).m = .09;
% 
%     %         rx      ry      rz
%     L(1).r = [0   0   0 ];
%     L(2).r = [0.068   0.006   -0.016];
%     L(3).r = [0   -0.070  0.014 ];
%     L(4).r = [0   0   -0.019];
%     L(5).r = [0   0   0 ];
%     L(6).r = [0   0   .032  ];
% 
%     %        Ixx     Iyy      Izz    Ixy     Iyz     Ixz
%     L(1).I = [0   0   0.35    0   0   0];
%     L(2).I = [.13   .524    .539    0     0   0];
%     L(3).I = [.066    .0125   .066    0   0   0];
%     L(4).I = [1.8e-3  1.8e-3  1.3e-3  0   0   0];
%     L(5).I = [.3e-3   .3e-3   .4e-3   0   0   0];
%     L(6).I = [.15e-3  .15e-3  .04e-3  0   0   0];
% 
%     L(1).Jm =  291e-6;
%     L(2).Jm =  409e-6;
%     L(3).Jm =  299e-6;
%     L(4).Jm =  35e-6;
%     L(5).Jm =  35e-6;
%     L(6).Jm =  35e-6;
% 
%     L(1).G =  -62.6111;
%     L(2).G =  107.815;
%     L(3).G =  -53.7063;
%     L(4).G =  76.0364;
%     L(5).G =  71.923;
%     L(6).G =  76.686;
end
%%%%%%------------------------------------------%%%%%%%


%%%%%%%------check input params------------%%%%%%%%%%%%
rob_basic_model = SerialLink(L, 'name', 'urhd', ...
        'manufacturer', 'Unimation', 'comment', 'urhdd');

tmp_joint_q = zeros(length(rob_basic_model.links), 1);
tmp_joint_dq = zeros(length(rob_basic_model.links), 1);
tmp_joint_ddq = zeros(length(rob_basic_model.links), 1);

tmp_dyn_hmat = rob_dyn_hmatrix_func(tmp_joint_q,...
                 tmp_joint_dq, tmp_joint_ddq);
tmp_dyn_baseh_mat = rob_dyn_hbasematrix_func(tmp_joint_q,...
                 tmp_joint_dq, tmp_joint_ddq);
size_regressor_mat = size(tmp_dyn_hmat);
size_base_regressor_mat = size(tmp_dyn_baseh_mat);
size_joints_limt = size(joints_limt);
assert( (size_joints_limt(1)==length(rob_basic_model.links))&&...
        (size_joints_limt(2)==6) ,'Error, please cheack joints limit' );

assert((size_regressor_mat(1)==length(rob_basic_model.links))&&(...
        size_base_regressor_mat(1)==length(rob_basic_model.links)),...
        'Error, please cheack dynamic regressor function');

if ~isempty(rob_dyn_params_h)
    assert(size_regressor_mat(2)==length(rob_dyn_params_h),...
        'Error, please cheack dynamic params');
elseif ~isempty(rob_dyn_params_baseh)
    assert(size_base_regressor_mat(2)==length(rob_dyn_params_baseh),...
        'Error, please cheack dynamic params');
end

%%%%%%%-------------------------------%%%%%%%%%%%%

%%%%%%%------set params---------------%%%%%%%%%%%%
rob_model_proterties = struct();
rob_model_proterties.basic_proterties = rob_basic_model;
rob_model_proterties.theoretically_dynamic_param = theoretically_dynamic_flg;
rob_model_proterties.dyn_hmat_func = rob_dyn_hmatrix_func;
rob_model_proterties.dyn_params_h = rob_dyn_params_h;
rob_model_proterties.dyn_params_baseh = rob_dyn_params_baseh;
rob_model_proterties.dyn_hbasemat_func = rob_dyn_hbasematrix_func;
rob_model_proterties.dyn_hmat_size = size_regressor_mat;
rob_model_proterties.dyn_hbasemat_size = size_base_regressor_mat;
rob_model_proterties.gravity = gravity;
rob_model_proterties.joints_limt = joints_limt;
%%%%%%%--------------------------------%%%%%%%%%%%%

end

