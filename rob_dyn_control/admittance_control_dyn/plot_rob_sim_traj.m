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

rob_model = get_model_params();
rob_model.basic_proterties.plot(out.position_velocity(1:100:end,1:6));
