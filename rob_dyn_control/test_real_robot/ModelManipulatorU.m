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

classdef ModelManipulatorU
    %MANIPULATOR_MODEL Identification & control of robotic dynamics

    
    properties(SetAccess=private) % getAccess=private
        % rob_model_proterties_: proterties of the target manipulator
        rob_model_proterties_;
    end
    
    methods
        function obj = ModelManipulatorU(varargin)
            %MANIPULATOR_MODEL Construct an instance of this class
            %   param_func_hd: handle of param function  s.t. get_model_params
            
            %   param_func_name: param_function name s.t. get_model_params
            
            input_param = inputParser;
            addParameter(input_param, 'param_func_hd', @get_model_params);
            input_param.parse(varargin{:});
            param_func_hd = input_param.Results.param_func_hd;
            
            % param_func = str2func(param_func_name);
            obj.rob_model_proterties_ = param_func_hd();
        end % function ModelManipulatorU

        function size_joint_state_data = check_joint_state_type(obj,...
                    joint_q, joint_dq, joint_ddq, varargin)
            % type check of the joint state data
            % input:
            %   joint_tor: sequential torque of joints
            %   joint_q： sequential joint position
            %   joint_dq： sequential joint velocity
            %   joint_ddq： sequential joint acceleration
            % output:
            %   stype size of the joint stata date
            input_param = inputParser;
            addParameter(input_param, 'joint_tor', []);
            parse(input_param, varargin{:}); 
            joint_tor = input_param.Results.joint_tor;
            
            size_joint_q = size(joint_q);
            size_joint_dq = size(joint_dq);
            size_joint_ddq = size(joint_ddq);
            
            if isempty(joint_tor)
                size_joint_tor = size_joint_q;
            else
                size_joint_tor = size(joint_tor);
            end
            
            assert((size_joint_q(1)==size_joint_dq(1))&&...
              (size_joint_dq(1)==size_joint_ddq(1))&&...
              (size_joint_tor(1)==size_joint_q(1))&&...
              (size_joint_q(2)==size_joint_dq(2))&&...
              (size_joint_dq(2)==size_joint_ddq(2))&&...
              (size_joint_tor(2)==size_joint_q(2))&&...
              (size_joint_q(2)==length(...
               obj.rob_model_proterties_.basic_proterties.links)),...
               "Error, please check the type of joint state");

            size_joint_state_data = size_joint_q;
            
        end % check_joint_state_type()
        
        function joint_tor = compute_joint_tor_theoretically(obj,...
                                   joint_q, joint_dq, joint_ddq)
            %get_joint_tor_sim: 
            % compute torque of joints based on 
            %       theoretically dynamic params      
            %  input:
            %   joint_q： sequential joint position
            %             e.g. [joint1_t1, joint2_t1,....jointn_t1;
            %                   joint1_t2, joint2_t2,,....jointn_t2;
            %                   ..................................;
            %                   joint1_tn, joint2_tn,....jointn_tn]
            %   joint_dq： sequential joint velocity
            %   joint_ddq： sequential joint acceleration
            %  output:
            %   joint_tor: sequential joint torque
            %              e.g.  [tor_joint1_t1, tor_joint2_t1,....tor_jointn_t1;
            %                     ..............................................;
            %                     tor_joint1_tn, tor_joint2_tn,....tor_jointn_tn]

            obj.check_joint_state_type(joint_q, joint_dq, joint_ddq);
            
            joint_tor = obj.rob_model_proterties_.basic_proterties.rne(...
                joint_q, joint_dq, joint_ddq, ...
                obj.rob_model_proterties_.gravity);
        end % function get_joint_tor_theoretically()

        function joint_tor = compute_joint_tor_regressor_mat(obj, ...
                               joint_q, joint_dq, joint_ddq, varargin)
            % compute torque of joints based on dynamics regressor matrix
            % input: 
            %   joint_q： sequential joint position
            %   joint_dq： sequential joint velocity
            %   joint_ddq： sequential joint acceleration
            %   use_base_dyn_params: 
            %           true--use base regressor params
            %           false--use regressor params
            %   dyn_params_regressor: 
            %           input temp dynamic params
            %           default:[] means use dynamic from obj.rob_model_proterties_
            %  output:
            %   joint_tor: sequential joint torque
            input_param = inputParser;
            addParameter(input_param, 'use_base_dyn_params', true);
            addParameter(input_param, 'dyn_params_regressor', []);
            parse(input_param, varargin{:}); 
            dyn_params_regressor = input_param.Results.dyn_params_regressor;
            use_base_dyn_params = input_param.Results.use_base_dyn_params;
            
            if isempty(dyn_params_regressor)
                if use_base_dyn_params
                    dyn_linear_params = obj.rob_model_proterties_.dyn_params_baseh;
                else
                    dyn_linear_params = obj.rob_model_proterties_.dyn_params_h;
                end
            else
                dyn_linear_params = dyn_params_regressor;
            end
            
            if use_base_dyn_params
                size_dyn_regressor_mat = obj.rob_model_proterties_.dyn_hbasemat_size;
            else
                size_dyn_regressor_mat = obj.rob_model_proterties_.dyn_hmat_size;
            end

            size_dyn_linear_params = size(dyn_linear_params);
            if(size_dyn_linear_params(1)==1)
                size_dyn_linear_params = size_dyn_linear_params';
            end
            assert(size_dyn_regressor_mat(2)==size_dyn_linear_params(1),...
            'Error, please check dynamic params or regressor matrix');

            size_joint_state = obj.check_joint_state_type(joint_q, joint_dq, joint_ddq);

            joint_tor = zeros(size_joint_state(1),...
                length(obj.rob_model_proterties_.basic_proterties.links));
            for i = 1: size_joint_state(1)
                dyn_regressor_mat = obj.compute_dynamic_regressor_mat(...
                    joint_q(i,:), joint_dq(i,:), joint_ddq(i,:), ...
                    'use_base_regressor_mat', use_base_dyn_params);
                joint_tor(i,:) = dyn_regressor_mat * dyn_linear_params;
            end

        end % function compute_joint_tor_regressor_mat()
        
        function dynamic_regressor_matrix = compute_dynamic_regressor_mat(obj,...
                      joint_q, joint_dq, joint_ddq, varargin)
            % compute dynamic regressor matrix
            % input:
            %     joint_q：joint position 
            %              e.g. [joint1_q,joint2_q,...jointn_q]
            %     joint_dq：joint velocity
            %              e.g. [joint1_dq,joint2_dq,...jointn_dq]
            %     joint_ddq：joint acceleration
            %              e.g. [joint1_ddq,joint2_ddq,...jointn_ddq]
            %     use_base_regressor_mat: 
            %                   true--use base regressor matrix
            %                   false--use regressor matrix
            % output:
            %     dynamic regressor matrix
            input_param = inputParser;
            addParameter(input_param, 'use_base_regressor_mat', true);
            parse(input_param, varargin{:}); 
            use_base_regressor_mat = ...
                input_param.Results.use_base_regressor_mat;
            
            size_joint_state =...
                obj.check_joint_state_type(joint_q, joint_dq, joint_ddq);
            assert(size_joint_state(1)==1,'Error, please input single time joint states')
            
            if use_base_regressor_mat
                dynamic_regressor_matrix = ...
                   obj.rob_model_proterties_.dyn_hbasemat_func(...
                           joint_q, joint_dq, joint_ddq);
            else
                dynamic_regressor_matrix = ...
                   obj.rob_model_proterties_.dyn_hmat_func(...
                           joint_q, joint_dq, joint_ddq);
            end
        end % function compute_dynamic_regressor_mat()
        
        function dyn_params = identify_dyn_params_lls(obj,...
                joint_tor, joint_q, joint_dq, joint_ddq, varargin)
            % identify dynamic params based on linear least square (H*params=Tor)
            %     which H is the regressor matrix,
            %     params is the dynamic params 
            %     Tor is the torque of joints
            % input:
            %   joint_tor: sequential torque of joints
            %   joint_q： sequential joint position
            %   joint_dq： sequential joint velocity
            %   joint_ddq： sequential joint acceleration
            %   use_base_regressor_mat: 
            %             true--use base regressor matrix
            %             false--use regressor matrix
            % output: 
            %     dyn_params: dynamic params 
            input_param = inputParser;
            addParameter(input_param, 'use_base_regressor_mat', true);
            parse(input_param, varargin{:}); 
            use_base_regressor_mat = ...
                input_param.Results.use_base_regressor_mat;
            
            size_joint_state = obj.check_joint_state_type(...
                   joint_q, joint_dq, joint_ddq,...
                   'joint_tor',joint_tor);
            
            if use_base_regressor_mat
                size_dyn_regressor_mat = obj.rob_model_proterties_.dyn_hbasemat_size;
            else
                size_dyn_regressor_mat = obj.rob_model_proterties_.dyn_hmat_size;
            end
            assert(size_dyn_regressor_mat(1)==size_joint_state(2),...
                'Error, please check the regress matrix or joint state');
            
            rob_dof = size_joint_state(2);
            data_sample_num = size_joint_state(1);
            % Solve Ax=b
            mat_a_regressor = zeros(size_dyn_regressor_mat(1)*data_sample_num,...
                size_dyn_regressor_mat(2));
            mat_b_tor = zeros( rob_dof*data_sample_num, 1);
            for i = 1: size_joint_state(1)
                mat_a_regressor((i-1)*rob_dof+1:i*rob_dof,:)...
                    = obj.compute_dynamic_regressor_mat(...
                       joint_q(i, :), joint_dq(i, :), joint_ddq(i, :), ...
                       'use_base_regressor_mat', use_base_regressor_mat);
                mat_b_tor((i-1)*rob_dof+1:i*rob_dof,:)...
                    = joint_tor(i, :)';
            end
            dyn_params = (pinv(mat_a_regressor'*mat_a_regressor)*mat_a_regressor')*mat_b_tor;
        end % function identify_dyn_params_lls()
        
        function [exciting_trajectory, exciting_trajectory_period_length,...
                  optimal_val] = compute_exciting_trajectory(obj, varargin)
            % compute the optimal excitation trajectory for
            %  dynaminc identification based on minimize condition number
            % input: 
            %   use_base_regressor_mat: 
            %       true--use base regressor matrix
            %       false--use regressor matrix
            %   max_period_length:
            %       maximum time length (unit s) of the exciting trajectory
            %   min_period_length:
            %       minimum time length (unit s) of the exciting trajectory
            %   period_sample_num:
            %       numbers of sampling at a period
            %   level_fourier_series:
            %       level of the fourier series
            %   solve_options: 
            %       options for optimal method
            % output:
            %   exciting_trajectory:
            %       return a cell which stored excitation trajectory function
            %       usage: exciting_trajectory_func{joint_n}{x_t} (x_t in [0,2*pi])
            %   exciting_trajectory_period_length:
            %       period length of the exciting trajectory
            %   optimal_val:
            %       value of the optimal result
            input_param = inputParser;
            addParameter(input_param, 'use_base_regressor_mat', true);
            addParameter(input_param, 'max_period_length', 60);
            addParameter(input_param, 'min_period_length', 10);
            addParameter(input_param, 'period_sample_num', 50);
            addParameter(input_param, 'level_fourier_series', 5);
            addParameter(input_param, 'solve_options',...
                optimoptions('fmincon','Algorithm','active-set'));
            parse(input_param, varargin{:}); 
            max_period_length = input_param.Results.max_period_length;
            min_period_length = input_param.Results.min_period_length;
            use_base_regressor_mat = input_param.Results.use_base_regressor_mat;
            period_sample_num = input_param.Results.period_sample_num;
            level_fourier_series = input_param.Results.level_fourier_series;
            solve_options = input_param.Results.solve_options;
            
            assert( (max_period_length>0) && (min_period_length>0) && (period_sample_num>1) && ...
                (level_fourier_series>1) ,(max_period_length>min_period_length),...
                'Error, please check the setting of the exciting trajectory')
            
            if use_base_regressor_mat
                size_dyn_regressor_mat = obj.rob_model_proterties_.dyn_hbasemat_size;
            else
                size_dyn_regressor_mat = obj.rob_model_proterties_.dyn_hmat_size;
            end
            rob_joint_num = length(obj.rob_model_proterties_.basic_proterties.links);
            size_joints_limt = size(obj.rob_model_proterties_.joints_limt);
            assert( (size_joints_limt(1)==rob_joint_num)&&...
                (size_joints_limt(2)==6),'Error, please check setting of the joints limt' );
            
            %%% dt_fs = max_period_length/(period_sample_num-1);%%%%%%%%%%%
            time_period_fs = sym('time_period_fs');
            % xt of the fourier series
            xt_fs = sym('xt_fs');
            % coefficients of the fourier series
            coef_fs = sym('coef_fs', [1, (2*level_fourier_series+1)]);
            num_coef_fs = length(coef_fs);
            fourier_series_sym = obj.compute_fourier_series(xt_fs,...
                coef_fs, time_period_fs);
            dt_fourier_series = diff(fourier_series_sym, xt_fs);
            ddt_fourier_series = diff(dt_fourier_series, xt_fs);
            fourier_series = matlabFunction(fourier_series_sym,...
                'Vars', {xt_fs, coef_fs, time_period_fs});
            dt_fourier_series = matlabFunction(dt_fourier_series,...
                'Vars', {xt_fs, coef_fs, time_period_fs});
            ddt_fourier_series = matlabFunction(ddt_fourier_series,...
                'Vars', {xt_fs, coef_fs, time_period_fs});

                function cond_mat_a = cond_regressor_full(joints_coef_fs)
                    period_length = joints_coef_fs(length(joints_coef_fs));
                    dt_fs = period_length/(period_sample_num-1);
                    cond_mat_a = zeros(size_dyn_regressor_mat(1)*...
                        period_sample_num, size_dyn_regressor_mat(2));
                    for i = 1:period_sample_num
                        tmp_joints_q = zeros(rob_joint_num, 1);
                        tmp_joints_dq = tmp_joints_q;
                        tmp_joints_ddq = tmp_joints_q;
                        for j = 1: rob_joint_num
                            tmp_joints_q(j) = fourier_series(dt_fs*(i-1),...
                                joints_coef_fs(:, ((j-1)*num_coef_fs+1):(j*num_coef_fs)), period_length);
                            tmp_joints_dq(j) = dt_fourier_series(dt_fs*(i-1),...
                                joints_coef_fs(:, ((j-1)*num_coef_fs+1):(j*num_coef_fs)), period_length);
                            tmp_joints_ddq(j) = ddt_fourier_series(dt_fs*(i-1),...
                                joints_coef_fs(:, ((j-1)*num_coef_fs+1):(j*num_coef_fs)), period_length);
                        end
                        cond_mat_a( ((i-1)*rob_joint_num+1):...
                            (i*rob_joint_num),:) = obj.compute_dynamic_regressor_mat(...
                                tmp_joints_q', tmp_joints_dq', tmp_joints_ddq', ...
                                'use_base_regressor_mat', use_base_regressor_mat);
                    end
                    cond_mat_a = cond(cond_mat_a);
                end % function cond_regressor_full()
                
                function [joints_limit_q_dq_ddq_per_fs, joints_limit_head_end] = compute_joints_limit(...
                        joints_coef_fs)
                    period_length = joints_coef_fs(length(joints_coef_fs));
                    dt_fs = period_length/(period_sample_num-1);
                    joints_limit_q_dq_ddq_per_fs = zeros(6*rob_joint_num*period_sample_num+2, 1);
                    joints_limit_head_end = zeros(2*rob_joint_num+2*rob_joint_num, 1);
                    
                    joints_limit_q_dq_ddq_per_fs(length(joints_limit_q_dq_ddq_per_fs))...
                        = min_period_length - period_length;
                    joints_limit_q_dq_ddq_per_fs(length(joints_limit_q_dq_ddq_per_fs) - 1)...
                        = period_length - max_period_length;
                    
                    for i = 1:period_sample_num
                        for j = 1: rob_joint_num
                            joint_j_q = fourier_series(dt_fs*(i-1),...
                                joints_coef_fs(:, ((j-1)*num_coef_fs+1):(j*num_coef_fs)), period_length);
                            joint_j_dq = dt_fourier_series(dt_fs*(i-1),...
                                joints_coef_fs(:, ((j-1)*num_coef_fs+1):(j*num_coef_fs)), period_length);
                            joint_j_ddq = ddt_fourier_series(dt_fs*(i-1),...
                                joints_coef_fs(:, ((j-1)*num_coef_fs+1):(j*num_coef_fs)), period_length);
                            joints_limit_q_dq_ddq_per_fs( 6*(i-1)*rob_joint_num+...
                                6*(j-1)+1) = obj.rob_model_proterties_.joints_limt(j, 1)-joint_j_q;
                            joints_limit_q_dq_ddq_per_fs( 6*(i-1)*rob_joint_num+...
                                6*(j-1)+2) = joint_j_q - obj.rob_model_proterties_.joints_limt(j, 2);
                            joints_limit_q_dq_ddq_per_fs( 6*(i-1)*rob_joint_num+...
                                6*(j-1)+3) = obj.rob_model_proterties_.joints_limt(j, 3)-joint_j_dq;
                            joints_limit_q_dq_ddq_per_fs( 6*(i-1)*rob_joint_num+...
                                6*(j-1)+4) = joint_j_dq - obj.rob_model_proterties_.joints_limt(j, 4);
                            joints_limit_q_dq_ddq_per_fs( 6*(i-1)*rob_joint_num+...
                                6*(j-1)+5) = obj.rob_model_proterties_.joints_limt(j, 5)-joint_j_ddq;
                            joints_limit_q_dq_ddq_per_fs( 6*(i-1)*rob_joint_num+...
                                6*(j-1)+6) = joint_j_ddq - obj.rob_model_proterties_.joints_limt(j, 6);
                            if i==1
                                % joints_limit_head_end( 3*(j-1)+1) = joint_j_q;
                                joints_limit_head_end( 2*(j-1)+1) = joint_j_dq;
                                joints_limit_head_end( 2*(j-1)+2) = joint_j_ddq;
                            elseif i==period_sample_num
%                                 joints_limit_head_end(length(joints_limit_head_end) -...
%                                     (3*(j-1)+1) ) = joint_j_q;
                                joints_limit_head_end(length(joints_limit_head_end) -...
                                    (2*(j-1)) ) = joint_j_dq;
                                joints_limit_head_end(length(joints_limit_head_end) -...
                                    (2*(j-1)+1) ) = joint_j_ddq;
                            end
                        end
                    end
                end % function compute_joints_limit()

            cof_init = rand(1, rob_joint_num*num_coef_fs+1);% +1 for time_period_fs
            cof_init(length(cof_init)) = min_period_length + (max_period_length-min_period_length)*rand();
            % [coefficients_result, fval] = fmincon();
            % @(cof) deal([minimum_limit_c(cof), minimum_limit_ceq(cof)])
            [coefficients_result, optimal_val] = fmincon(@cond_regressor_full,...
                cof_init,...
                [],[],[],[],[],[],...
                @compute_joints_limit,...
                solve_options);
            size_coefficients_result = size(coefficients_result);
            if size_coefficients_result(1)>size_coefficients_result(2)
                coefficients_result = coefficients_result';
            end
            exciting_trajectory_period_length = coefficients_result(length(coefficients_result));
            exciting_trajectory = repmat({@(x)x},1,rob_joint_num);
            for k = 1:rob_joint_num
                exciting_trajectory{k} =  matlabFunction( subs(fourier_series_sym,...
                    [coef_fs, time_period_fs],...
                    [coefficients_result(:, ((k-1)*num_coef_fs+1):(k*num_coef_fs) ),...
                     exciting_trajectory_period_length] ) );
            end
        end %function compute_exciting_trajectory()

    end % method of the class
    
    methods(Static)

        function result_fourier_series = compute_fourier_series(input_t,...
                   input_coefficients, time_period)
            % finite fourier series for optimal robot excitation
            % reference:
            % Swevers, Jan, et al. "Optimal robot excitation and 
            % identification." IEEE transactions on robotics and 
            % automation 13.5 (1997): 730-740.
            % input: 
            %   input_t: 
            %      time domain coordinates (input_t in [0,2*pi])
            %   coefficients: 
            %      coefficients of the fourier series
            %      e.g. [a0, a1, b1, a2, b2, a3, b3,......]
            %   time_period：
            %      period length of the fourier series 
            % output: 
            %      result of the fourier series
            size_coefficients = size(input_coefficients);
            assert((size_coefficients(1)==1)&&rem(size_coefficients(2),2),...
                'Error, please check params of the fourier series')
            result_fourier_series = input_coefficients(1);
            frequency_w = (2*pi)/time_period;
            for i = 2 : 2 : size_coefficients(2)
                result_fourier_series = result_fourier_series + ...
                    (input_coefficients(i)/(frequency_w*(i/2)))*...
                    sin((frequency_w*(i/2))*input_t) - ...
                    (input_coefficients(i+1)/(frequency_w*(i/2)))*...
                    cos((frequency_w*(i/2))*input_t);
            end

        end % function get_func_fourier_series()

    end % methods(Static) of the class
% author:  foxchys  email:chy_s@outlook.com
end % class ModelManipulatorU

