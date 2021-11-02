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
function [sys,x0,str,ts,simStateCompliance] = compute_admittance(t,x,u,flag)

switch flag

  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;

  case 1
    % sys=mdlDerivatives(t,x,u);
    sys=[];

  case 2
    sys=mdlUpdate(t,x,u);

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
% position--velocity--external force--M(damping) B(damping) K(stiffness)--
% t pre
sizes.NumDiscStates  = 1;
sizes.NumOutputs     = rob_dof; % position increment
% Kp(pid Kp)
% position--velocity--external force--M(damping) B(damping) K(stiffness)
sizes.NumInputs      = 3*rob_dof+4; 
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);

% position--velocity--external force--M B K--delta t
x0  = zeros(1, sizes.NumDiscStates); % 

str = [];

ts  = [-1, 0];

simStateCompliance = 'UnknownSimState';

end % mdlInitializeSizes

% function sys=mdlDerivatives(t,x,u)
% 
% end % mdlDerivatives

function sys=mdlUpdate(t,x,u)

% sys = zeros(1, length(u)+1); % 

% sys(1:(end-1)) = u;

% update delta t
sys = t;

end % mdlUpdate

function sys=mdlOutputs(t,x,u)
% u position--velocity--external force--M(damping) B(damping) K(stiffness)
global rob_model; % TODO check joint limit---------------------------
global rob_dof;

delta_t = t-x(end);
coefficient_Kp = u(end-3);
% M(damping) B(damping) K(stiffness)
coefficient_M = u(end-2); 
coefficient_B = u(end-1);
coefficient_K = u(end);
coefficient_B = 2*coefficient_B*sqrt(coefficient_M*coefficient_Kp);% 
sys = zeros(rob_dof, 1);
for i = 1:rob_dof

    cmd_ddq_i = ( u(2*rob_dof+i) -...
        coefficient_B*(u(rob_dof+i)-0) - ...
        coefficient_K*(u(i)-0) )/coefficient_M;
    % Backward Euler Discrete-Time Integrator
    delta_dq =0 + delta_t*cmd_ddq_i;
    delta_q = u(rob_dof+i) + delta_t*delta_dq;

    sys(i) = delta_q;
end


end % mdlOutputs

function sys=mdlTerminate(t,x,u)

sys=[];

end % mdlTerminate