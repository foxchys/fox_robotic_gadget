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
function [sys,x0,str,ts,simStateCompliance] = dynamic_feedforward(t,x,u,flag)

switch flag

  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;

  case 1
    % sys=mdlDerivatives(t,x,u);
    sys=[];

  case 2
    % sys=mdlUpdate(t,x,u);
    sys=[];

  case 3
    sys=mdlOutputs(t,x,u);

  case 4
    % sys=mdlGetTimeOfNextVarHit(t,x,u);
    sys = [];

  case 9
    sys=mdlTerminate(t,x,u);

  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

end % process_rob_model


function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
global rob_model;
rob_model = get_model_params();
global rob_dof;
rob_dof = length(rob_model.basic_proterties.links);

sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0; % position(rad)--velocity(rad/s)
sizes.NumOutputs     = rob_dof; % position--torque
sizes.NumInputs      = 3*rob_dof; % position--velocity--acceleration of every joint
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);

x0  = []; % 

str = [];

ts  = [-1, 0];

simStateCompliance = 'UnknownSimState';

end % mdlInitializeSizes

% function sys=mdlDerivatives(t,x,u)

% end % mdlDerivatives

% function sys=mdlUpdate(t,x,u)
% 
% end % mdlUpdate

function sys=mdlOutputs(t,x,u)
global rob_model;
global rob_dof;
base_regressor_mat = rob_model.dyn_hbasemat_func(...
    u(1:rob_dof)', u(rob_dof+1:2*rob_dof)',...
    u(2*rob_dof+1:end)');
sys = base_regressor_mat*rob_model.dyn_params_baseh;

end % mdlOutputs

function sys=mdlTerminate(t,x,u)

sys=[];

end % mdlTerminate